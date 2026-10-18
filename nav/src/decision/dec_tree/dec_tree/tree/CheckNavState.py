import py_trees
from py_trees.common import Status
from .parameter import NAV_STATUS


class CheckNavState(py_trees.behaviour.Behaviour):
    """Read asynchronous task results without blocking a tactical tree tick."""
    def __init__(self, name, nav, node):
        super().__init__(name)
        self.nav, self.node = nav, node
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key('nav_status', py_trees.common.Access.WRITE)
        self.blackboard.nav_status = NAV_STATUS.NOGOAL

    def update(self):
        self.blackboard.nav_status = self.nav.status
        return Status.SUCCESS
