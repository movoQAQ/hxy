#!/usr/bin/env python3
"""Physical swerve regression in its own Gazebo partition and ROS domain.

Requires sourced gazebo + nav workspaces and the swerve_tracking_probe target.
Never sends commands to the user's navigation/Gazebo partition.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from sdformat_tools.xmacro4sdf import XMLMacro4sdf


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenarios', nargs='+', default=['straight', 'hairpin', 'recovery', 'overshoot'])
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[3]
    package = Path(__file__).resolve().parents[1]
    probe = workspace/'build/nav_executor/swerve_tracking_probe'
    if not probe.exists():
        raise SystemExit('Build nav_executor with BUILD_TESTING enabled first.')
    folder = Path(tempfile.mkdtemp(prefix='qp-swerve-regression-'))
    print('Artifacts:', folder, flush=True)
    macro = XMLMacro4sdf()
    macro.set_xml_file(str(Path(get_package_share_directory('pb2025_robot_description'))/
                           'resource/xmacro/pb2025_sentry_robot.sdf.xmacro'))
    macro.generate({'global_initial_color': 'red'})
    model = ET.fromstring(macro.to_string()).find('model')
    model.set('name', 'tracking_robot')
    model.find('pose').text = '0 0 0.88 0 0 0'
    drive = model.find("plugin[@filename='SwerveDrive']")
    actuators = {key: float(drive.findtext(key)) for key in ['wheel_radius','wheel_acceleration',
                 'steering_gain','steering_velocity','steering_acceleration']}
    print('Actual model actuator settings:', json.dumps(actuators), flush=True)
    world = ET.fromstring('''<sdf version="1.9"><world name="tracking">
      <physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <model name="ground"><static>true</static><link name="ground"><collision name="ground">
      <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
      </collision></link></model></world></sdf>''')
    world.find('world').append(model)
    world_file = folder/'world.sdf'
    ET.ElementTree(world).write(world_file)
    reports = []
    for index, scenario in enumerate(args.scenarios):
        env = dict(os.environ, ROS_DOMAIN_ID='186', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                   GZ_PARTITION=f'qp_tracking_{os.getpid()}_{index}', ROS_LOG_DIR=str(folder/'ros_logs'))
        processes, logs = [], []
        def start(name, command):
            stream = (folder/f'{scenario}-{name}.log').open('w'); logs.append(stream)
            proc = subprocess.Popen(command, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            processes.append(proc)
            return proc
        try:
            start('gz', ['gz','sim','-s','-r',str(world_file)])
            start('bridge', ['ros2','run','ros_gz_bridge','parameter_bridge',
                             '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                             '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'])
            result = folder/f'{scenario}.json'
            child = start('controller', [str(probe),'--ros-args','--params-file',str(package/'config/mpc.yaml'),
                                        '-p',f'scenario:={scenario}','-p',f'result_file:={result}'])
            code = child.wait(timeout=150)
            if not result.exists():
                raise AssertionError(f'{scenario} failed without metrics; logs in {folder}')
            report = json.loads(result.read_text()); reports.append(report)
            print(json.dumps(report), flush=True)
            if code or not report['success']:
                raise AssertionError(f'{scenario} failed; logs in {folder}')
        finally:
            for child in reversed(processes):
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGINT)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL); child.wait()
            for stream in logs:
                stream.close()
    (folder/'summary.json').write_text(json.dumps(dict(actuators=actuators,scenarios=reports),indent=2)+'\n')
    print('PASS:', folder/'summary.json', flush=True)


if __name__ == '__main__':
    main()
