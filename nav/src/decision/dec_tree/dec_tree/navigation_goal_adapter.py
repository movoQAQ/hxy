"""Nonblocking tactical intent adapter for the NavigateTask action.

Only this adapter owns action handles. Old callbacks are tagged with a local
sequence so a late result/cancel acknowledgement cannot overwrite a newer task.
"""
import copy
import threading

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_task_interfaces.action import NavigateTask

from .tree.parameter import NAV_STATUS


class TacticalGoalAdapter(Node):
    def __init__(self):
        super().__init__('tactical_goal_adapter')
        self.client = ActionClient(self, NavigateTask, 'navigate_task')
        self._lock = threading.RLock()
        self._sequence = 0
        self._key = None
        self._handle = None
        self._status = NAV_STATUS.NOGOAL
        self.feedback = None
        self.last_error = ''

    @property
    def status(self):
        with self._lock:
            return self._status

    @staticmethod
    def intent_key(goal, task_name, priority, mode):
        p, q = goal.pose.position, goal.pose.orientation
        return (goal.header.frame_id, round(p.x, 4), round(p.y, 4), round(p.z, 4),
                round(q.x, 5), round(q.y, 5), round(q.z, 5), round(q.w, 5),
                str(task_name), int(priority), int(mode))

    def send(self, goal, task_name='', priority=50, mode=NavigateTask.Goal.NAVIGATE):
        key = self.intent_key(goal, task_name, priority, mode)
        with self._lock:
            if key == self._key:
                return True  # Includes terminal results: BT ticks do not resend a completed intent.
            if not self.client.server_is_ready():
                self.last_error = 'NavigateTask server unavailable'
                return False
            self._sequence += 1
            sequence = self._sequence
            self._key = key
            self._handle = None
            self._status = NAV_STATUS.RUNNING
            self.feedback = None
            self.last_error = ''
            request = NavigateTask.Goal()
            request.target = copy.deepcopy(goal)
            request.target.header.stamp = self.get_clock().now().to_msg()
            request.goal_mode, request.priority = int(mode), int(priority)
            request.task_name = str(task_name)
            future = self.client.send_goal_async(
                request, feedback_callback=lambda msg: self._feedback(sequence, msg))
            future.add_done_callback(lambda f: self._accepted(sequence, f))
            return True

    def _feedback(self, sequence, message):
        with self._lock:
            if sequence == self._sequence:
                self.feedback = message.feedback

    def _accepted(self, sequence, future):
        try:
            handle = future.result()
        except Exception as error:
            with self._lock:
                if sequence == self._sequence:
                    self._status, self.last_error = NAV_STATUS.ERROR, str(error)
            return
        with self._lock:
            if sequence != self._sequence:
                if handle.accepted:
                    handle.cancel_goal_async()  # Cancels this UUID only, never a newer goal.
                return
            if not handle.accepted:
                self._status, self.last_error = NAV_STATUS.FAILED, 'Navigation task rejected'
                return
            self._handle = handle
            handle.get_result_async().add_done_callback(lambda f: self._result(sequence, f))

    def _result(self, sequence, future):
        with self._lock:
            if sequence != self._sequence:
                return
            try:
                response = future.result()
                result = response.result
                self.last_error = result.message if not result.success else ''
                if response.status == GoalStatus.STATUS_SUCCEEDED and result.success:
                    self._status = NAV_STATUS.SUCCEEDED
                elif response.status == GoalStatus.STATUS_CANCELED or result.result_code in (
                        NavigateTask.Result.CANCELLED, NavigateTask.Result.PREEMPTED):
                    self._status = NAV_STATUS.CANCELED
                else:
                    self._status = NAV_STATUS.FAILED
            except Exception as error:
                self._status, self.last_error = NAV_STATUS.ERROR, str(error)
            self._handle = None

    def cancel(self):
        with self._lock:
            self._sequence += 1  # Also invalidates a pending acceptance callback.
            handle, self._handle = self._handle, None
            self._key = None
            self._status = NAV_STATUS.CANCELED
            if handle is not None:
                handle.cancel_goal_async()
