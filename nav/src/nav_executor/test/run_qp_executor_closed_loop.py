#!/usr/bin/env python3
"""ROS integration smoke test with an lagged planar plant, isolated domain 188.

Publishes synthetic raw costmaps/TF/odometry and a goal, starts the real launch,
and requires execution success without a controller failure. Not a Gazebo or
hardware dynamics test. Run from a sourced nav workspace.
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
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from map_server.msg import Costmap
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class Plant(Node):
    def __init__(self):
        super().__init__('qp_mpc_integration_plant')
        self.x, self.y = 2.0, 2.0
        self.vx = self.vy = 0.0
        self.ux = self.uy = 0.0
        self.commands = 0
        self.success_speed = None
        self.moving_inside_goal = False
        self.last = time.monotonic()
        self.success = False
        self.failures = []
        maps_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.maps = [(self.create_publisher(Costmap, '/global_costmap/costmap_raw', maps_qos), 'map'),
                     (self.create_publisher(Costmap, '/local_costmap/costmap_raw', maps_qos), 'odom')]
        self.odom = self.create_publisher(Odometry, '/odom', 1)
        self.goal = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.command_sub = self.create_subscription(Twist, '/cmd_vel', self.command, 1)
        self.state_sub = self.create_subscription(String, '/nav_executor/execution_state', self.state, 10)
        self.error_sub = self.create_subscription(String, '/nav_executor/execution_error',
                                                  lambda msg: self.failures.append(msg.data) if msg.data else None, 10)
        self.static_tf = StaticTransformBroadcaster(self)
        self.dynamic_tf = TransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.rotation.w = 1.0
        self.static_tf.sendTransform(transform)
        self.timer = self.create_timer(0.02, self.tick)
        self.map_timer = self.create_timer(0.1, self.publish_maps)
        self.publish_maps()

    def command(self, msg):
        self.ux, self.uy = msg.linear.x, msg.linear.y
        self.commands += int(math.hypot(self.ux, self.uy) > 1e-6)
        if not math.isfinite(self.ux+self.uy) or math.hypot(self.ux, self.uy) > 1.401:
            raise AssertionError('invalid published velocity')

    def state(self, msg):
        self.success = self.success or msg.data == 'SUCCEEDED'
        if msg.data == 'SUCCEEDED':
            self.success_speed = math.hypot(self.vx,self.vy)

    def publish_maps(self):
        for publisher, frame in self.maps:
            msg = Costmap()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame
            msg.info.width = msg.info.height = 180
            msg.info.resolution = 0.05
            msg.info.origin.orientation.w = 1.0
            msg.data = [255]*(180*180)
            msg.current = True
            msg.robot_base_frame = 'base_link_fake'
            msg.robot_radius, msg.inflation_radius, msg.cost_scaling_factor = 0.15, 0.55, 5.0
            msg.track_unknown_space = True
            for i in range(16):
                angle = i*2*math.pi/16
                x, y = 0.15*math.cos(angle), 0.15*math.sin(angle)
                x += 0.01 if x > 0 else -0.01 if x < 0 else 0
                y += 0.01 if y > 0 else -0.01 if y < 0 else 0
                msg.footprint.append(Point(x=x, y=y, z=0.0))
            publisher.publish(msg)

    def tick(self):
        current = time.monotonic()
        elapsed = min(current-self.last, 0.05)
        self.last = current
        beta = math.exp(-elapsed/0.25)
        travel = 0.25*(1-beta)
        self.x += travel*self.vx+(elapsed-travel)*self.ux
        self.y += travel*self.vy+(elapsed-travel)*self.uy
        self.vx = beta*self.vx+(1-beta)*self.ux
        self.vy = beta*self.vy+(1-beta)*self.uy
        if math.hypot(self.x-3,self.y-2)<0.05 and math.hypot(self.vx,self.vy)>0.03:
            self.moving_inside_goal = True
        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id = stamp, 'odom'
        transform.child_frame_id = 'base_link_fake'
        transform.transform.translation.x, transform.transform.translation.y = self.x, self.y
        transform.transform.rotation.w = 1.0
        self.dynamic_tf.sendTransform(transform)
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id = stamp, 'odom'
        msg.child_frame_id = 'base_link_fake'
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = self.vx, self.vy
        self.odom.publish(msg)

    def send_goal(self):
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'map'
        msg.pose.position.x, msg.pose.position.y = 3.0, 2.0
        msg.pose.orientation.w = 1.0
        self.goal.publish(msg)


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '188':
        raise SystemExit('Use isolated ROS_DOMAIN_ID=188 for this synthetic test.')
    with tempfile.TemporaryDirectory(prefix='qp-executor-test-') as temporary:
        log_path = Path(temporary)/'navigation.log'
        rclpy.init()
        node = Plant()
        with log_path.open('w') as output:
            child = subprocess.Popen(['ros2', 'launch', 'nav_executor', 'navigation_launch.py',
                                      'use_sim_time:=false'], stdout=output, stderr=subprocess.STDOUT,
                                     start_new_session=True)
            try:
                began = time.monotonic()
                sent = False
                while time.monotonic()-began < 20.0 and not node.success:
                    rclpy.spin_once(node, timeout_sec=0.01)
                    if child.poll() is not None:
                        raise AssertionError('navigation exited unexpectedly')
                    if not sent and time.monotonic()-began > 2 and node.goal.get_subscription_count():
                        node.send_goal()
                        sent = True
                assert sent and node.success, 'goal execution did not succeed'
                # Ignore normal lifecycle reasons (new goal / replacing execution path).
                errors = [e for e in node.failures if e not in ('New goal', 'Replacing execution path')]
                assert not errors, errors
                assert node.commands > 0, 'no nonzero QP commands observed'
                assert node.success_speed <= 0.031, 'goal accepted before actual stopping'
                assert math.hypot(node.x-3, node.y-2) < 0.051
                report = dict(success=node.success, x=node.x, y=node.y,
                              nonzero_commands=node.commands, success_speed_mps=node.success_speed,
                              moving_inside_goal=node.moving_inside_goal,
                              final_error_m=math.hypot(node.x-3,node.y-2))
                print(json.dumps(report, indent=2))
            except BaseException:
                print(log_path.read_text())
                raise
            finally:
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGINT)
                    try:
                        child.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait()
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
