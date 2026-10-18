import math

import py_trees
from geometry_msgs.msg import PoseStamped
from py_trees.common import Status


class PubGoal(py_trees.behaviour.Behaviour):
    """Publish navigation intent; the supervisor owns preemption and planning."""
    def __init__(self, name, nav):
        super().__init__(name)
        self.nav = nav
        self.blackboard = self.attach_blackboard_client()
        for key in ('goal', 'dec_now'):
            self.blackboard.register_key(key, py_trees.common.Access.READ)
        self.blackboard.register_key('nav_status', py_trees.common.Access.WRITE)

    def update(self):
        try:
            target = self.blackboard.goal
            task_name = self.blackboard.dec_now or ''
        except KeyError:
            return Status.FAILURE
        if target is None:
            return Status.FAILURE
        goal = PoseStamped()
        goal.header.frame_id = target.get('frame_id', 'map')
        goal.pose.position.x, goal.pose.position.y = float(target['x']), float(target['y'])
        yaw = float(target.get('yaw', 0.0))
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if not self.nav.send(goal, task_name, target.get('priority', 50), target.get('goal_mode', 0)):
            return Status.RUNNING  # Server startup is polled on later BT ticks.
        self.blackboard.nav_status = self.nav.status
        return Status.SUCCESS
