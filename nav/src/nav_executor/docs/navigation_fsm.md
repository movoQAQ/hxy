# 导航任务接口与运动 FSM

本次重构保留现有战术 py_trees、TaskManager、异步 PathPlanner、拓扑/栅格规划器和 qpOASES MPC。
新增 NavigateTask Action、MotionFSM、SafetyMonitor、RecoveryController。战术树只提交目标意图，
不再依赖 BasicNavigator、Nav2 TaskResult 或 Nav2 取消/清图接口。

## 数据与控制流

```mermaid
flowchart TD
  BT[战术 py_trees] --> Adapter[TacticalGoalAdapter]
  Adapter --> Action[NavigateTask Action]
  RViz[RViz /goal_pose] --> TM[TaskManager]
  Action --> TM
  TM --> Planner[异步 PathPlanner]
  Planner --> Validation[最新代价地图验证]
  Validation --> Motion[PathExecutor / MotionFSM]
  Route[RouteMonitor: 偏离、前方封路] --> TM
  Safety[SafetyMonitor: 物理卡住] --> Motion
  Motion --> Recovery[RecoveryController: 全向脱困]
  Recovery --> TM
  Motion --> Gate[唯一命令出口 / 安全门控]
  Gate --> Cmd[/cmd_vel]
```

任务状态与运动状态独立。脱困和小陀螺期间，TaskManager 保留同一个 goal_id，撤销当前路径并暂停普通规划。
恢复结束后增加 plan_generation，从当前位置重新规划；旧 worker 结果不能重新接管控制。

## 运动状态

| 值 | 状态 | 行为 |
|---|---|---|
| 0 | IDLE | 发布零速度，等待已验证路径 |
| 1 | FOLLOW | 原 qpOASES MPC，使用新鲜实测速度 |
| 2 | HOLD | NAVIGATE 到点后零速度；HOLD 模式在到点位置做限速位置保持 |
| 3 | PREPARE_SPIN | 先制动，确认实测平移/角速度低于阈值 |
| 4 | SPIN | 有界角速度；退出后申请新路径 |
| 5 | ESCAPE | 物理卡住后，在安全方向低速短距离撤离 |
| 6 | HAZARD_RECOVERY | 在危险位置附近搜索局部安全位置，验证完整撤离走廊 |

优先级为：紧急停止/底盘故障/状态失效 > 恢复动作 > 小陀螺 > 跟随/保持。
只由 PathExecutor 创建一个 `/cmd_vel` publisher，MPC、保持、小陀螺和恢复控制共用最终出口。
`fake_baselink` 转换平移坐标时保留角速度，让上位机小陀螺指令可以传递到底盘桥接。
本版小陀螺使用上位机角速度接口，没有实现实车下位机独占模式的交权确认协议。

## NavigateTask

类型：`nav_task_interfaces/action/NavigateTask`，名称：`/navigate_task`。

- Goal：`target`、`goal_mode`、`priority`、`task_name`。
- `goal_mode=0`：NAVIGATE，到点停车；`goal_mode=1`：HOLD，到点后限速保持该位置。
- 数值更大的 priority 优先；相同优先级的新意图可抢占；低优先级请求被拒绝。
- Result code：0 成功、1 失败、2 被新目标抢占、3 被取消。抢占通过旧 Action 的 abort + code=2 返回。
- Feedback：服务端 goal_id、plan_generation、任务/运动/恢复状态、距终点的直线距离、重规划原因。
  距离尚不可用时为 -1。枚举常量随 Action 一起生成，客户端无需手写状态数字。
- 任务状态：0 IDLE、1 WAITING_FOR_MAP、2 PLANNING、3 PATH_READY、4 SUCCEEDED、5 FAILED。
- 恢复状态：0 NONE、1 ESCAPE、2 HAZARD。
- 重规划原因：0 NONE、1 PATH_DEVIATION、2 EXECUTION_STALLED、3 PATH_BLOCKED、
  4 RECOVERY_COMPLETED、5 SPIN_RELEASED、6 HAZARD。

Action 的规划/执行失败有重试上限和整体期限。没有安全脱困方向、恢复超时、紧急停止等为终止失败，
不会无限重试恢复动作。新的显式目标或 replan 才能重新启动已终止任务。
RViz 的目标也交给同一个 TaskManager；无效/重复 RViz 目标不会取消正在执行的 Action。

```bash
source /opt/ros/jazzy/setup.bash
source /home/lyu/hxy/nav/install/setup.bash
ros2 action send_goal /navigate_task nav_task_interfaces/action/NavigateTask \
  "{target: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 9.5}, orientation: {w: 1.0}}}, goal_mode: 0, priority: 50, task_name: patrol}" --feedback

ros2 service call /nav_executor/set_spin std_srvs/srv/SetBool '{data: true}'
ros2 service call /nav_executor/set_spin std_srvs/srv/SetBool '{data: false}'
ros2 service call /nav_executor/cancel std_srvs/srv/Trigger '{}'
ros2 service call /nav_executor/replan std_srvs/srv/Trigger '{}'
```

## 战术树适配

`ros2 run dec_tree root` 继续运行原来的战术条件与巡逻逻辑。TacticalGoalAdapter 用异步 Action：
相同位置、姿态、任务名、优先级和模式在连续 tick 中只发送一次；已经完成的相同意图也不会重复发送。
不同意图交给服务端抢占。每个回调带本地序号，旧结果/旧取消确认不能覆盖新任务。
Home、Patrol 的初始化不再循环等待取消；坐标、等待和随机/顺序巡逻逻辑保留。
点位 YAML 可以选填 `yaw`、`priority`（默认 50）、`goal_mode`（默认 0）。

原战术树有几处裁判字段名称与当前消息不一致，`TacticalReferee` 提供以下兼容名称：

| 战术字段 | 当前 Referee 字段 |
|---|---|
| enemy_hero_pos | catch_hero |
| enemy_engineer_pos | catch_engineer |
| fortress_enemy | defend_fortress |
| bumpy_road | bumpy_exist_enemy |

原 `pilot_cmd` 在当前消息中没有等价字段，改为显式订阅 `/tactical/pilot_cmd`（std_msgs/Int32）：
0 无指令、1 前往中央高地、2 进攻敌方基地。默认 0，话题名可通过 receiver 的 pilot_cmd_topic 参数修改。

## 安全与参数

参数在 `bringup/params/params.yaml` 及 standalone `nav_executor/config/nav_executor.yaml` 中。
`config/mpc.yaml` 仍然是现有 MPC 的独立参数来源。

- `task.*`：max_plan_attempts=5（连续失败尝试）、timeout=120 秒、rviz_priority=10。
- `safety.*`：有平移命令、实测低速且位置持续不变才判定卡住；新鲜度中断会重置计时。
  `braking_acceleration` 用于最终命令扫掠范围检查，不修改 MPC 调参。
- `recovery.*`：默认移动 0.35 m、速度上限 0.15 m/s、加速度上限 0.3 m/s²、6 秒期限。
  使用包住完整足迹的外接圆检查整段走廊，拒绝障碍、未知区域、地图外；每拍复查动态走廊。
  如果机器人已经与障碍相交、找不到全程安全的撤离方向，会停车并报告失败，不假设可以穿透障碍。
- `spin.*`：角速度 0.5 rad/s、角加速度 0.5 rad/s²；平移/角速度低于 0.05 才可开始旋转；准备期限 3 秒。
- `/nav_executor/emergency_stop`、`/nav_executor/chassis_fault`（std_msgs/Bool）为保持式安全输入。
  true 立即制动；false 解除禁止，但不会自行恢复已终止的任务。
- `/nav_executor/motion_state`、`execution_state`、`execution_error`、`state` 为可观测状态。

RouteMonitor 继续负责路径偏离和前方阻塞。原 `route_monitor.stall_*` 参数保留兼容声明，
物理卡住改由 `safety.*` 配置，避免只重规划而不脱困的循环。

## 构建与验证

```bash
cd /home/lyu/hxy/nav
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to nav_executor dec_tree
source install/setup.bash
ctest --test-dir build/nav_executor --output-on-failure
pytest -q src/decision/dec_tree/test/test_navigation_goal_adapter.py
ROS_DOMAIN_ID=189 ROS_LOG_DIR=/tmp/nav_fsm_ros_logs \
  python3 src/nav_executor/test/run_supervisor_integration.py
```

集成测试使用独立 ROS 域、合成代价地图和简化运动模型，覆盖 Action 完成、优先级、抢占、取消、
小陀螺释放后重规划、卡住后全向脱困、无安全方向时停车及紧急停止。它不替代 Gazebo 或实车动力学验证。
设计参考：[HWSentryNav26 状态机](https://github.com/Polyacetone/HWSentryNav26/blob/main/nav_executor/src/path_executor/state/state_machine.cpp)。
