#!/usr/bin/env python3

import argparse
import json
import math
import re
from pathlib import Path


NUMBER = r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)'
CONTROL_PATTERN = re.compile(
    rf'\[(?P<time>\d+\.\d+)\] \[ibvs_controller\]: '
    rf'err=\[\s*(?P<eu>{NUMBER}),\s*(?P<ev>{NUMBER})\].*?'
    rf'qd_max=(?P<qd_max>{NUMBER}).*?'
    rf'sigma_min=(?P<sigma_min>{NUMBER})'
)
DEADZONE_PATTERN = re.compile(
    rf'\[(?P<time>\d+\.\d+)\] \[ibvs_controller\]: '
    rf'Dead zone \(err=(?P<error>{NUMBER})px\)'
)
PATH_TOLERANCE_PX = 1.5


def path_length(points):
    return sum(
        math.hypot(
            current[0] - previous[0],
            current[1] - previous[1],
        )
        for previous, current in zip(points, points[1:])
    )


def point_to_segment_distance(point, start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.hypot(
            point[0] - start[0], point[1] - start[1])

    projection = (
        (point[0] - start[0]) * dx
        + (point[1] - start[1]) * dy
    ) / length_squared
    projection = min(1.0, max(0.0, projection))
    closest = (
        start[0] + projection * dx,
        start[1] + projection * dy,
    )
    return math.hypot(
        point[0] - closest[0], point[1] - closest[1])


def simplify_path(points, tolerance):
    # 使用像素容差消除整数质心形成的楼梯状伪路径。
    if len(points) <= 2:
        return list(points)

    keep = {0, len(points) - 1}
    segments = [(0, len(points) - 1)]

    while segments:
        start_index, end_index = segments.pop()
        start = points[start_index]
        end = points[end_index]
        farthest_index = None
        farthest_distance = -1.0

        for index in range(start_index + 1, end_index):
            distance = point_to_segment_distance(
                points[index], start, end)
            if distance > farthest_distance:
                farthest_distance = distance
                farthest_index = index

        if (
                farthest_index is not None
                and farthest_distance > tolerance):
            keep.add(farthest_index)
            segments.append((start_index, farthest_index))
            segments.append((farthest_index, end_index))

    return [points[index] for index in sorted(keep)]


def lateral_deviations(points):
    # 横向偏差直接衡量轨迹偏离起点到终点直线的程度。
    if len(points) <= 1:
        return [0.0]
    return [
        point_to_segment_distance(point, points[0], points[-1])
        for point in points
    ]


def analyze_log_text(text):
    samples = []
    first_deadzone = None

    for line in text.splitlines():
        control_match = CONTROL_PATTERN.search(line)
        if control_match:
            samples.append({
                'time': float(control_match.group('time')),
                'eu': float(control_match.group('eu')),
                'ev': float(control_match.group('ev')),
                'qd_max': float(control_match.group('qd_max')),
                'sigma_min': float(control_match.group('sigma_min')),
            })

        if first_deadzone is None:
            deadzone_match = DEADZONE_PATTERN.search(line)
            if deadzone_match:
                first_deadzone = {
                    'time': float(deadzone_match.group('time')),
                    'error': float(deadzone_match.group('error')),
                }

    if not samples:
        raise ValueError('log does not contain controller error samples')

    error_points = [
        (sample['eu'], sample['ev']) for sample in samples
    ]
    error_norms = [
        math.hypot(eu, ev) for eu, ev in error_points
    ]

    # 对静态目标，误差轨迹与绿色特征轨迹只相差一个常量。
    raw_path_length = path_length(error_points)
    simplified_points = simplify_path(
        error_points, PATH_TOLERANCE_PX)
    simplified_path_length = path_length(simplified_points)
    straight_displacement = math.hypot(
        error_points[-1][0] - error_points[0][0],
        error_points[-1][1] - error_points[0][1],
    )
    path_efficiency = (
        straight_displacement / simplified_path_length
        if simplified_path_length > 0.0
        else 1.0
    )
    deviations = lateral_deviations(error_points)
    rms_lateral_deviation = math.sqrt(
        sum(value * value for value in deviations) / len(deviations)
    )

    convergence_time = None
    deadzone_error = None
    if first_deadzone is not None:
        convergence_time = first_deadzone['time'] - samples[0]['time']
        deadzone_error = first_deadzone['error']

    green_stale_count = text.count(
        'Green feature stale or missing')
    moving_target_stale_count = text.count(
        'Moving target stale or missing')
    initial_target_wait_count = text.count(
        'Waiting for initial target')
    error_increase_steps = sum(
        current > previous + 1e-9
        for previous, current in zip(error_norms, error_norms[1:])
    )

    # 初始等待目标是正常启动状态，不计入失败条件。
    reached_deadzone = first_deadzone is not None
    success = (
        reached_deadzone
        and green_stale_count == 0
        and moving_target_stale_count == 0
    )

    return {
        'success': success,
        'sample_count': len(samples),
        'initial_error_px': error_norms[0],
        'last_control_error_px': error_norms[-1],
        'deadzone_error_px': deadzone_error,
        'convergence_time_sec': convergence_time,
        'raw_path_length_px': raw_path_length,
        'simplified_path_length_px': simplified_path_length,
        'simplified_point_count': len(simplified_points),
        'straight_displacement_px': straight_displacement,
        'path_efficiency': path_efficiency,
        'rms_lateral_deviation_px': rms_lateral_deviation,
        'max_lateral_deviation_px': max(deviations),
        'peak_commanded_joint_speed': max(
            sample['qd_max'] for sample in samples),
        'minimum_sigma': min(
            sample['sigma_min'] for sample in samples),
        'error_increase_steps': error_increase_steps,
        'green_stale_warnings': green_stale_count,
        'moving_target_stale_warnings': moving_target_stale_count,
        'initial_target_wait_warnings': initial_target_wait_count,
    }


def format_summary(result):
    def display(value, digits=3):
        if value is None:
            return 'N/A'
        return f'{value:.{digits}f}'

    lines = [
        f"success: {result['success']}",
        f"samples: {result['sample_count']}",
        f"initial_error_px: {display(result['initial_error_px'])}",
        (
            'last_control_error_px: '
            f"{display(result['last_control_error_px'])}"
        ),
        f"deadzone_error_px: {display(result['deadzone_error_px'])}",
        (
            'convergence_time_sec: '
            f"{display(result['convergence_time_sec'])}"
        ),
        f"raw_path_length_px: {display(result['raw_path_length_px'])}",
        (
            'simplified_path_length_px: '
            f"{display(result['simplified_path_length_px'])}"
        ),
        (
            'simplified_point_count: '
            f"{result['simplified_point_count']}"
        ),
        (
            'straight_displacement_px: '
            f"{display(result['straight_displacement_px'])}"
        ),
        f"path_efficiency: {display(result['path_efficiency'], 4)}",
        (
            'rms_lateral_deviation_px: '
            f"{display(result['rms_lateral_deviation_px'])}"
        ),
        (
            'max_lateral_deviation_px: '
            f"{display(result['max_lateral_deviation_px'])}"
        ),
        (
            'peak_commanded_joint_speed: '
            f"{display(result['peak_commanded_joint_speed'])}"
        ),
        f"minimum_sigma: {display(result['minimum_sigma'], 4)}",
        f"error_increase_steps: {result['error_increase_steps']}",
        f"green_stale_warnings: {result['green_stale_warnings']}",
        (
            'moving_target_stale_warnings: '
            f"{result['moving_target_stale_warnings']}"
        ),
        (
            'initial_target_wait_warnings: '
            f"{result['initial_target_wait_warnings']}"
        ),
    ]
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(
        description='Analyze an IBVS controller ROS log.')
    parser.add_argument('log_file', type=Path)
    parser.add_argument(
        '--json', action='store_true',
        help='Print machine-readable JSON.')
    args = parser.parse_args()

    result = analyze_log_text(
        args.log_file.read_text(encoding='utf-8', errors='replace'))
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(format_summary(result))


if __name__ == '__main__':
    main()
