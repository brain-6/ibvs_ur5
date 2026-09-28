#!/usr/bin/env python3
"""一个静态 trial：完整启动、观测、判定、保存、关闭；不做参数搜索。"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from ament_index_python.packages import get_package_share_directory
import yaml

from trial_metrics import TrialJudge


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# Humble uint8 constants may be bytes; received level values are integers.
WARN_LEVEL = Log.WARN[0] if isinstance(Log.WARN, bytes) else Log.WARN


class Recorder(Node):
    def __init__(self, output):
        super().__init__('phase4_recorder')
        self.judge = TrialJudge(time.monotonic())
        self.raw = {}
        self.q = None
        self.q_time = None
        self.initial_q = None
        self.locked_target = None
        self.initial_error = None
        self.previous_sequence = None
        self.sequence_gaps = 0
        self.decode_errors = []
        self.controller_stale_warnings = 0
        self.clock_events = []
        self.clock_pair = (time.time(), time.monotonic())
        self.csv_file = (output / 'errors.csv').open('w', newline='')
        self.writer = csv.DictWriter(self.csv_file, fieldnames=[
            'mono', 'relative_seconds', 'feature_sequence', 'feature_callback_mono',
            'observation_age_seconds', 'target_u', 'target_v', 'feature_u', 'feature_v',
            'error_u', 'error_v', 'error_norm', 'detected_red_u', 'detected_red_v'])
        self.writer.writeheader()
        self.create_subscription(String, '/ibvs/observation', self.observation, 100)
        self.create_subscription(JointTrajectory,
                                 '/joint_trajectory_controller/joint_trajectory',
                                 self.command, 10)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_subscription(Point, '/target_centroid',
                                 lambda msg: self.raw_point('red', msg), 10)
        self.create_subscription(Point, '/feature_centroid',
                                 lambda msg: self.raw_point('green', msg), 10)
        self.create_subscription(Log, '/rosout', self.log, 100)
        self.controllers = self.create_client(ListControllers,
                                               '/controller_manager/list_controllers')
        self.parameters = self.create_client(GetParameters,
                                              '/ibvs_controller/get_parameters')

    def raw_point(self, name, msg):
        self.raw[name] = (time.monotonic(), [msg.x, msg.y])

    def joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in JOINT_NAMES):
            self.q = [positions[name] for name in JOINT_NAMES]
            self.q_time = time.monotonic()

    def command(self, msg):
        if msg.points and self.judge.first_command is None:
            self.initial_q = self.q
            self.judge.command(time.monotonic())

    def observation(self, msg):
        now = time.monotonic()
        if self.judge.end_reason is not None:
            return
        try:
            value = json.loads(msg.data)
            target, feature, error = value['target'], value['feature'], value['error']
            if any(len(pair) != 2 for pair in (target, feature, error)):
                raise ValueError('observation vectors must have length 2')
            if not all(math.isfinite(x) for pair in (target, feature, error) for x in pair):
                raise ValueError('nonfinite coordinate')
            if any(abs(feature[i] - target[i] - error[i]) > 1e-9 for i in range(2)):
                raise ValueError('inconsistent error vector')
            sequence = int(value['feature_sequence'])
            source_time = float(value['feature_mono'])
            age = now - source_time
            if self.previous_sequence is not None and sequence != self.previous_sequence + 1:
                self.sequence_gaps += 1
                self.judge.interrupt()
            self.previous_sequence = sequence
            if self.locked_target is None:
                self.locked_target = target
            elif self.locked_target != target:
                raise ValueError('static controller target changed')
            norm = math.hypot(*error)
            red = self.raw.get('red', (None, [None, None]))[1]
            t0 = self.judge.first_command
            self.writer.writerow({
                'mono': now, 'relative_seconds': None if t0 is None else now - t0,
                'feature_sequence': sequence, 'feature_callback_mono': source_time,
                'observation_age_seconds': age,
                'target_u': target[0], 'target_v': target[1],
                'feature_u': feature[0], 'feature_v': feature[1],
                'error_u': error[0], 'error_v': error[1], 'error_norm': norm,
                'detected_red_u': red[0], 'detected_red_v': red[1],
            })
            if t0 is not None and self.initial_error is None:
                self.initial_error = norm
            self.judge.observe(now, norm, sample_age=age)
        except (ValueError, TypeError, KeyError) as exc:
            self.decode_errors.append({'mono': now, 'reason': str(exc)})
            self.judge.interrupt()

    def log(self, msg):
        if (msg.name == 'ibvs_controller' and msg.level >= WARN_LEVEL and
                'stale or missing' in msg.msg and self.judge.first_command is not None and
                self.judge.end_reason is None):
            # 日志已节流，这里是警告条数，不能叫逐帧 stale 次数。
            self.controller_stale_warnings += 1

    def poll(self):
        wall, mono = time.time(), time.monotonic()
        delta = (wall - self.clock_pair[0]) - (mono - self.clock_pair[1])
        if abs(delta) > 0.25:
            self.clock_events.append({'mono': mono, 'wall_minus_mono_step_seconds': delta})
        self.clock_pair = wall, mono
        self.judge.poll(mono)

    def inputs_ready(self):
        now = time.monotonic()
        return (self.q is not None and now - self.q_time <= 0.5 and
                all(name in self.raw and now - self.raw[name][0] <= 0.5
                    for name in ('red', 'green')))


def stop_owned_processes(processes):
    """只关闭本 trial 启动的进程组；不用全局 pkill。"""
    cleanup = []
    for name, process, handle in reversed(processes):
        sent = []
        for sig, seconds in ((signal.SIGINT, 8), (signal.SIGTERM, 4), (signal.SIGKILL, 2)):
            if process.poll() is not None:
                break
            try:
                os.killpg(process.pid, sig)
                sent.append(sig.name)
                process.wait(timeout=seconds)
            except ProcessLookupError:
                break
            except subprocess.TimeoutExpired:
                continue
        handle.close()
        cleanup.append({'name': name, 'pid': process.pid,
                        'exit_code': process.poll(), 'signals_sent': sent})
    return cleanup


def run(args):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)  # 拒绝覆盖任何已有 trial。
    repo = args.repo.resolve()
    result = {'trial_id': output.name, 'factor': args.factor, 'level': args.level,
              'seed': None, 'protocol': 'static-pixel-hold-v1',
              'requested_controller_parameters': None, 'effective_controller_parameters': None,
              'end_reason': 'execution_error', 'geometric_converged': False,
              'gui': args.gui, 'use_sim_time_note': 'Controller clock semantics unchanged.',
              'observation_basis': 'Controller feature callback with actual latched target; not image acquisition time.',
              'timing_basis': 'Receiver monotonic time; delayed observation age also checked.',
              'protocol_limits': {'threshold_px_strict': 5, 'hold_seconds': 3,
                                  'observation_seconds': 30, 'freshness_seconds': 0.5,
                                  'startup_seconds': 30, 'clock_step_reporting_seconds': 0.25}}
    processes = []
    recorder = None
    ros_initialized = False

    def start(name, command):
        handle = (output / f'{name}.log').open('w')
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True, cwd=repo)
        processes.append((name, process, handle))
        print(f'START {name} pid={process.pid}', flush=True)

    def pump():
        rclpy.spin_once(recorder, timeout_sec=0.02)
        recorder.poll()
        for name, process, _ in processes:
            if process.poll() is not None:
                raise RuntimeError(f'process_exited:{name}:{process.returncode}')

    def wait_for(predicate):
        while recorder.judge.end_reason is None:
            if predicate():
                return
            pump()
        raise RuntimeError(recorder.judge.end_reason)

    def service_reply(client, request):
        wait_for(client.service_is_ready)
        future = client.call_async(request)
        wait_for(future.done)
        return future.result()

    try:
        config = yaml.safe_load(args.params.read_text())['ibvs_controller']['ros__parameters']
        if config.get('static_target') is not True:
            raise ValueError('This recorder protocol only supports static_target=true')
        if config.get('deadzone_px') != 5.0 or config.get('feature_timeout_sec') != 0.5:
            raise ValueError('Protocol requires deadzone_px=5 and feature_timeout_sec=0.5')
        config['record_observations'] = True
        result['requested_controller_parameters'] = config
        applied_file = output / 'controller_parameters.yaml'
        applied_file.write_text(yaml.safe_dump({'ibvs_controller': {'ros__parameters': config}}))
        result['git_commit'] = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
        result['git_status'] = subprocess.check_output(
            ['git', '--no-optional-locks', 'status', '--porcelain'], cwd=repo, text=True)
        if result['git_status'] and not args.allow_dirty:
            raise RuntimeError('dirty_repository: use --allow-dirty only for development smoke tests')
        world = Path(get_package_share_directory('ibvs_ur5')) / 'worlds/visual_servo.sdf'
        result['world_sha256'] = hashlib.sha256(world.read_bytes()).hexdigest()
        result['urdf_sha256'] = hashlib.sha256(Path('/tmp/ur5_gazebo.urdf').read_bytes()).hexdigest()
        result['ros_domain_id'] = os.environ.get('ROS_DOMAIN_ID', '0 (default)')
        result['detector_parameters'] = {'write_debug_image': False, 'log_interval_sec': 1.0}
        rclpy.init()
        ros_initialized = True
        recorder = Recorder(output)
        # 给 ROS graph 少量发现时间，拒绝混入已运行的同项目节点。
        until = time.monotonic() + 1.0
        while time.monotonic() < until:
            rclpy.spin_once(recorder, timeout_sec=0.05)
        conflicts = set(recorder.get_node_names()) & {
            'ibvs_controller', 'target_detector', 'controller_manager', 'robot_state_publisher'}
        if conflicts:
            raise RuntimeError('existing_nodes:' + ','.join(sorted(conflicts)))
        recorder.judge = TrialJudge(time.monotonic())
        start('gazebo', ['ros2', 'launch', 'ibvs_ur5', 'gazebo_ur5.launch.py',
                         f'gui:={str(args.gui).lower()}'])
        while True:
            reply = service_reply(recorder.controllers, ListControllers.Request())
            states = {item.name: item.state for item in reply.controller}
            if all(states.get(name) == 'active' for name in
                   ('joint_state_broadcaster', 'joint_trajectory_controller')):
                result['controllers'] = states
                break
            pump()
        start('detector', ['ros2', 'run', 'ibvs_ur5', 'detect_target.py', '--ros-args',
                           '-p', 'write_debug_image:=false', '-p', 'log_interval_sec:=1.0'])
        wait_for(recorder.inputs_ready)
        result['precontrol_q'] = recorder.q
        result['precontrol_red'] = recorder.raw['red'][1]
        result['precontrol_green'] = recorder.raw['green'][1]
        # 只拒绝明显不同的关节起点；不以有噪声的质心偏差筛掉实验。
        expected_q = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        result['initial_q_max_deviation_rad'] = max(abs(a - b) for a, b in zip(recorder.q, expected_q))
        if result['initial_q_max_deviation_rad'] > 0.05:
            raise RuntimeError('initial_joint_pose_mismatch')
        start('controller', ['ros2', 'run', 'ibvs_ur5', 'ibvs_controller.py', '--ros-args',
                             '--params-file', str(applied_file)])
        request = GetParameters.Request(names=list(config))
        reply = service_reply(recorder.parameters, request)
        effective = {name: parameter_value_to_python(value)
                     for name, value in zip(config, reply.values)}
        result['effective_controller_parameters'] = effective
        if effective != config:
            raise RuntimeError('effective_parameter_mismatch')
        while recorder.judge.end_reason is None:
            pump()
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = repr(exc)
        if recorder is not None:
            recorder.judge.finish(time.monotonic(), 'execution_error')
        print('ERROR ' + repr(exc), flush=True)
    finally:
        # 先冻结指标，再清理，关闭进程的时间不算入收敛或最终窗口。
        if recorder is not None:
            result.update(recorder.judge.summary())
            result.update({'initial_q_at_first_command': recorder.initial_q,
                           'latched_target': recorder.locked_target,
                           'initial_observed_error_px': recorder.initial_error,
                           'controller_stale_warning_count': recorder.controller_stale_warnings,
                           'observation_sequence_gap_count': recorder.sequence_gaps,
                           'decode_errors': recorder.decode_errors,
                           'clock_events': recorder.clock_events})
        result['cleanup'] = stop_owned_processes(processes)
        if recorder is not None:
            recorder.csv_file.close()
            recorder.destroy_node()
        if ros_initialized and rclpy.ok():
            rclpy.shutdown()
        anomalies = []
        for field in ('stale_intervals', 'invalid_samples', 'interrupted_intervals',
                      'controller_stale_warning_count', 'observation_sequence_gap_count',
                      'decode_errors', 'clock_events'):
            if result.get(field):
                anomalies.append(field)
        if result.get('error'):
            anomalies.append('execution_error')
        if result['effective_controller_parameters'] is None:
            anomalies.append('effective_parameters_unverified')
        if any(item['exit_code'] != 0 for item in result['cleanup']):
            anomalies.append('process_cleanup_error')
        result['anomalies'] = anomalies
        result['success'] = bool(result.get('geometric_converged')) and not anomalies
        result['formal_candidate'] = result['success'] and not result.get('git_status', 'unknown')
        (output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
        print(json.dumps({key: result.get(key) for key in
                          ('trial_id', 'end_reason', 'success', 'formal_candidate',
                           'convergence_seconds', 'final_error_mean_px', 'anomalies')},
                         ensure_ascii=False), flush=True)
    return 0 if result['success'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repo', type=Path, default=Path.cwd())
    parser.add_argument('--params', type=Path, default=Path(
        get_package_share_directory('ibvs_ur5')) / 'config/phase4_static_baseline.yaml')
    parser.add_argument('--factor', default='baseline')
    parser.add_argument('--level', default='nominal')
    parser.add_argument('--gui', action='store_true', help='默认仅服务端；各正式 trial 保持同一模式。')
    parser.add_argument('--allow-dirty', action='store_true', help='仅开发冒烟，不能记为正式干净版本。')
    return run(parser.parse_args())


if __name__ == '__main__':
    raise SystemExit(main())
