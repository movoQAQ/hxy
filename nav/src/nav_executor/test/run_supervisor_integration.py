#!/usr/bin/env python3
"""Exercise the real action server and motion FSM with an isolated synthetic plant.

Run from a sourced workspace with ROS_DOMAIN_ID=189. This validates scheduling,
control ownership and collision rejection, not Gazebo/hardware dynamics.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from map_server.msg import Costmap
from nav_task_interfaces.action import NavigateTask
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
import yaml

from run_qp_executor_closed_loop import Plant


class SupervisorPlant(Plant):
    def __init__(self):
        self.motion = 'IDLE'
        self.motions = []
        self.freeze_follow = False
        self.hazard = False
        self.max_spin = 0.0
        self.max_speed = 0.0
        self.feedback = None
        super().__init__()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.motion_sub = self.create_subscription(String, '/nav_executor/motion_state', self.on_motion, qos)
        self.client = ActionClient(self, NavigateTask, '/navigate_task')
        self.spin_client = self.create_client(SetBool, '/nav_executor/set_spin')
        self.estop = self.create_publisher(Bool, '/nav_executor/emergency_stop', 1)

    def on_motion(self, msg):
        self.motion = msg.data
        if not self.motions or self.motions[-1] != msg.data:
            self.motions.append(msg.data)

    def command(self, msg):
        super().command(msg)
        self.max_spin = max(self.max_spin, abs(msg.angular.z))
        self.max_speed = max(self.max_speed, math.hypot(msg.linear.x, msg.linear.y))

    def tick(self):
        ux, uy = self.ux, self.uy
        if self.freeze_follow and self.motion == 'FOLLOW':
            self.ux = self.uy = 0.0
        super().tick()
        self.ux, self.uy = ux, uy

    def publish_maps(self):
        for publisher, frame in self.maps:
            msg = Costmap()
            msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), frame
            msg.info.width = msg.info.height = 180
            msg.info.resolution = 0.05
            msg.info.origin.orientation.w = 1.0
            data = [255] * (180 * 180)
            if self.hazard and frame == 'odom':
                cx, cy = int(self.x / .05), int(self.y / .05)
                for y in range(cy-5, cy+6):
                    for x in range(cx-5, cx+6):
                        data[y * 180 + x] = 0
            msg.data = data
            msg.current = True
            msg.robot_base_frame = 'base_link_fake'
            msg.robot_radius, msg.inflation_radius, msg.cost_scaling_factor = .15, .55, 5.
            msg.track_unknown_space = True
            for i in range(16):
                a = i * 2 * math.pi / 16
                x, y = .15 * math.cos(a), .15 * math.sin(a)
                x += .01 if x > 0 else -.01 if x < 0 else 0
                y += .01 if y > 0 else -.01 if y < 0 else 0
                msg.footprint.append(Point(x=x, y=y, z=0.))
            publisher.publish(msg)

    def target(self, x=None, y=None, priority=50, name='test'):
        request = NavigateTask.Goal()
        request.target.header.frame_id = 'map'
        request.target.pose.position.x = self.x + 1 if x is None else float(x)
        request.target.pose.position.y = self.y if y is None else float(y)
        request.target.pose.orientation.w = 1.
        request.priority, request.task_name = priority, name
        return request


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '189':
        raise SystemExit('Run this isolated test with ROS_DOMAIN_ID=189')
    rclpy.init()
    node = SupervisorPlant()
    report = {}
    with tempfile.TemporaryDirectory(prefix='navigation-fsm-') as folder:
        folder = Path(folder)
        params = yaml.safe_load((Path(get_package_share_directory('bringup'))/'params/params.yaml').read_text())
        for producer in ('global_costmap', 'local_costmap'):
            cfg = params[producer]['ros__parameters']
            cfg.update(use_sim_time=False, robot_radius=.15, footprint='', footprint_padding=.01,
                       cost_scaling_factor=5., inflation_radius=.55, costmap_timeout=1.)
        cfg = params['nav_executor']['ros__parameters']
        cfg.update(use_sim_time=False, control_timeout=.2, xy_goal_tolerance=.05, stopped_velocity_tolerance=.03,
                   goal_confirmations=3, planning_timeout=3.)
        cfg['safety']['stall_duration'] = .6
        cfg['recovery']['timeout'] = 8.
        params['topology_planner']['ros__parameters']['use_sim_time'] = False
        (folder/'params.yaml').write_text(yaml.safe_dump(params))
        mpc = yaml.safe_load((Path(get_package_share_directory('nav_executor'))/'config/mpc.yaml').read_text())
        mpc['nav_executor']['ros__parameters'].update(control_timeout=.2, xy_goal_tolerance=.05)
        mpc['nav_executor']['ros__parameters']['MPC']['max_speed'] = 1.
        (folder/'mpc.yaml').write_text(yaml.safe_dump(mpc))
        log = (folder/'navigation.log').open('w')
        child = subprocess.Popen(['ros2', 'launch', 'nav_executor', 'navigation_launch.py',
                                  'use_sim_time:=false', 'params_file:='+str(folder/'params.yaml'),
                                  'controller_params_file:='+str(folder/'mpc.yaml')],
                                 stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        def pump(predicate, timeout=10):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.01)
                if child.poll() is not None:
                    raise AssertionError('Supervisor exited')
                if predicate():
                    return
            raise AssertionError('Timed out; motion='+node.motion)
        def result(future, timeout=10):
            pump(future.done, timeout)
            return future.result()
        def goal(request):
            return result(node.client.send_goal_async(request, feedback_callback=lambda msg: setattr(node, 'feedback', msg.feedback)))
        def spin(enabled):
            response = result(node.spin_client.call_async(SetBool.Request(data=enabled)))
            assert response.success, response.message
        try:
            pump(lambda: node.client.server_is_ready() and node.spin_client.service_is_ready(), 15)
            # A real NavigateTask completes with measured stopping and HOLD, without Nav2.
            handle = goal(node.target(3, 2, name='complete'))
            assert handle.accepted
            completed = result(handle.get_result_async(), 15)
            assert completed.result.success, completed.result.message
            pump(lambda: node.motion == 'HOLD')
            report['completion'] = dict(success=True, position=[node.x, node.y], speed=math.hypot(node.vx,node.vy))
            # A lower priority task cannot preempt; an equal-priority intent can.
            first = goal(node.target(6, 2, 80, 'first'))
            assert first.accepted
            assert not goal(node.target(1, 2, 10, 'low')).accepted
            second = goal(node.target(6, 3, 80, 'second'))
            assert second.accepted
            assert result(first.get_result_async()).result.result_code == NavigateTask.Result.PREEMPTED
            assert result(second.cancel_goal_async()).goals_canceling
            assert result(second.get_result_async()).result.result_code == NavigateTask.Result.CANCELLED
            pump(lambda: node.motion == 'IDLE')
            report['priority_preemption_cancel'] = True
            # Spin retains the tactical goal, then resumes only with a new plan generation.
            handle = goal(node.target(6, 2, name='spin'))
            pump(lambda: node.motion == 'FOLLOW' and node.feedback is not None)
            identity = (node.feedback.goal_id, node.feedback.plan_generation)
            spin(True);pump(lambda: node.motion == 'SPIN')
            pump(lambda: node.max_spin > .05)
            spin(False);pump(lambda: node.motion == 'FOLLOW' and node.feedback.plan_generation > identity[1])
            assert node.feedback.goal_id == identity[0]
            result(handle.cancel_goal_async());result(handle.get_result_async())
            report['spin_release_replans_same_goal'] = True
            # Freeze only normal follow motion. Escape physically moves, then requests a fresh route.
            node.feedback = None;node.freeze_follow = True
            handle = goal(node.target(name='stuck'))
            pump(lambda: node.motion == 'FOLLOW' and node.feedback is not None)
            identity = (node.feedback.goal_id,node.feedback.plan_generation)
            pump(lambda: node.motion == 'ESCAPE',8)
            node.freeze_follow = False
            pump(lambda: node.motion == 'FOLLOW' and node.feedback.plan_generation > identity[1],10)
            assert node.feedback.goal_id == identity[0]
            result(handle.cancel_goal_async());result(handle.get_result_async())
            report['physical_escape_replans_same_goal'] = True
            # Dynamic hazard at the current footprint: no blindly reversing through lethal cells.
            handle = goal(node.target(name='blocked'))
            pump(lambda: node.motion == 'FOLLOW')
            node.hazard = True
            outcome = result(handle.get_result_async(),8)
            assert not outcome.result.success
            assert 'safe' in outcome.result.message.lower() or 'obstacle' in outcome.result.message.lower(), outcome.result.message
            pump(lambda: abs(node.ux)+abs(node.uy) < 1e-9)
            node.hazard = False
            report['surrounded_recovery_stops'] = True
            # Emergency stop has authority over every motion state.
            handle = goal(node.target(name='estop'))
            pump(lambda: node.motion == 'FOLLOW')
            node.estop.publish(Bool(data=True))
            assert not result(handle.get_result_async()).result.success
            pump(lambda: abs(node.ux)+abs(node.uy) < 1e-9)
            node.estop.publish(Bool(data=False))
            report['emergency_stop'] = True
            report['motion_states'] = node.motions
            report['max_command_speed'] = node.max_speed
            print(json.dumps(report, indent=2),flush=True)
        except BaseException:
            log.flush();print((folder/'navigation.log').read_text()[-12000:],flush=True)
            print(json.dumps(report,indent=2),flush=True)
            raise
        finally:
            os.killpg(child.pid, signal.SIGINT)
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait()
            log.close();node.destroy_node();rclpy.shutdown()

if __name__ == '__main__':
    main()
