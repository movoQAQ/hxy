"""Regression checks for late action responses and per-tick intent deduplication."""
from concurrent.futures import Future
import threading
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav_task_interfaces.action import NavigateTask
from dec_tree.navigation_goal_adapter import TacticalGoalAdapter
from dec_tree.tree.parameter import NAV_STATUS


def adapter():
    client = TacticalGoalAdapter.__new__(TacticalGoalAdapter)
    client._lock = threading.RLock()
    client._sequence, client._key, client._handle = 0, None, None
    client._status = NAV_STATUS.NOGOAL
    client.last_error, client.feedback = '', None
    client.client = Mock()
    client.client.server_is_ready.return_value = True
    client.get_clock = Mock()
    client.get_clock.return_value.now.return_value.to_msg.return_value = Time()
    return client


def pose(x):
    value = PoseStamped();value.header.frame_id='map';value.pose.position.x=float(x);value.pose.orientation.w=1.
    return value


def handle():
    value=Mock();value.accepted=True;value.get_result_async.return_value=Future();return value


def test_duplicate_ticks_do_not_resubmit_even_after_success():
    client=adapter();accepted=Future();client.client.send_goal_async.return_value=accepted
    assert client.send(pose(1),'patrol')
    for _ in range(100):assert client.send(pose(1),'patrol')
    active=handle();accepted.set_result(active)
    outcome=NavigateTask.Result(success=True,result_code=NavigateTask.Result.OK)
    active.get_result_async.return_value.set_result(Mock(status=GoalStatus.STATUS_SUCCEEDED,result=outcome))
    assert client.status==NAV_STATUS.SUCCEEDED
    assert client.send(pose(1),'patrol');assert client.client.send_goal_async.call_count==1


def test_late_old_acceptance_cancels_only_its_own_uuid():
    client=adapter();old,new=Future(),Future();client.client.send_goal_async.side_effect=[old,new]
    client.send(pose(1),'patrol');client.send(pose(2),'home')
    newer=handle();new.set_result(newer)
    older=handle();old.set_result(older)
    older.cancel_goal_async.assert_called_once();newer.cancel_goal_async.assert_not_called()
    assert client.status==NAV_STATUS.RUNNING
    newer.get_result_async.return_value.set_result(Mock(status=GoalStatus.STATUS_SUCCEEDED,
        result=NavigateTask.Result(success=True,result_code=NavigateTask.Result.OK)))
    assert client.status==NAV_STATUS.SUCCEEDED


def test_cancel_invalidates_pending_acceptance():
    client=adapter();pending=Future();client.client.send_goal_async.return_value=pending
    client.send(pose(1));client.cancel();old=handle();pending.set_result(old)
    old.cancel_goal_async.assert_called_once();assert client.status==NAV_STATUS.CANCELED


def test_tactical_tree_constructs_with_the_action_adapter():
    import py_trees
    from rclpy.qos import QoSProfile
    from dec_tree.root import create_root_tree
    py_trees.blackboard.Blackboard.clear()
    nav = Mock(status=NAV_STATUS.NOGOAL)
    root = create_root_tree(Mock(), QoSProfile(depth=10), nav)
    assert root.name == 'root'
    assert any(node.name == 'pub_goal' for node in root.iterate())


def test_patrol_switch_submits_intent_without_blocking_cancel_loop():
    import py_trees
    from dec_tree.tree.BasicBehaviour.Patrol import Patrol
    from dec_tree.tree.PubGoal import PubGoal
    py_trees.blackboard.Blackboard.clear()
    nav = Mock(status=NAV_STATUS.RUNNING)
    nav.send.return_value = True
    patrol = Patrol('test_patrol', Mock(), nav, random=0)
    writer = py_trees.blackboard.Client(name='fixture')
    for key in ('/yaml/test_patrol', '/yaml/blood_limit', '/goal', '/dec_now', '/nav_status', '/Referee'):
        writer.register_key(key, py_trees.common.Access.WRITE)
    writer.set('/yaml/test_patrol', [{'x': 1., 'y': 2.}])
    writer.set('/yaml/blood_limit', 201)
    writer.set('/nav_status', NAV_STATUS.RUNNING)
    assert patrol.update() == py_trees.common.Status.SUCCESS
    assert PubGoal('test_pub', nav).update() == py_trees.common.Status.SUCCESS
    nav.cancel.assert_not_called()
    nav.isTaskComplete.assert_not_called()
    nav.cancelTask.assert_not_called()
    assert nav.send.call_count == 1


def test_existing_tactical_tree_ticks_with_current_referee_fields():
    import py_trees
    from rclpy.qos import QoSProfile
    from referee_msg.msg import Referee
    from dec_tree.root import create_root_tree
    from dec_tree.tactical_referee import TacticalReferee
    py_trees.blackboard.Blackboard.clear()
    nav=Mock(status=NAV_STATUS.NOGOAL);nav.send.return_value=True
    root=create_root_tree(Mock(),QoSProfile(depth=10),nav)
    writer=py_trees.blackboard.Client(name='referee_fixture')
    for key in ('Referee','region_int','region_area'):
        writer.register_key(key,py_trees.common.Access.WRITE)
    writer.Referee=TacticalReferee(Referee())
    writer.region_int=-1;writer.region_area=0
    root.tick_once()
    root.tick_once()
    assert nav.send.called
    nav.cancelTask.assert_not_called()


def test_tactical_referee_uses_explicit_pilot_command_and_current_wire_names():
    from referee_msg.msg import Referee
    from dec_tree.tactical_referee import TacticalReferee
    message=Referee(catch_hero=2,catch_engineer=1,defend_fortress=1,bumpy_exist_enemy=1)
    view=TacticalReferee(message,pilot_cmd=2)
    assert (view.enemy_hero_pos,view.enemy_engineer_pos,view.fortress_enemy,view.bumpy_road)==(2,1,1,1)
    assert view.pilot_cmd==2
