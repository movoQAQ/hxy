# Topology + tdt YAstar + MinimumSnap

当前是 `nav_executor` 直接调用的独立 C++ 库，不是 Nav2 planner plugin。

调用链：`nav_executor -> TopologyGlobalPlanner -> GridPlanner -> YAstar -> MinimumSnap -> nav_msgs/Path -> MPC`。
`topo` 是区域与 connector 的 wrapper：决定区域路线、connector 等待点/出口和单向通行；tdt 负责每段栅格搜索和平滑。`use_topology: false` 时仍会用 YAstar + MinimumSnap 直接规划到目标。

## 地图接口

`map_server::Costmap2D`、`Costmap.msg` 和动态障碍消息统一使用 tdt 方向的 traversability：

|值|含义|
|---|---|
|0|障碍|
|255|空闲|
|3..254|可通行的膨胀梯度，越小越危险|
|2|机器人中心不可进入的内切膨胀带|
|1|未知标记，必须按配置解释|

`1`、`2` 是本工程的语义扩展，不能直接按 tdt 的“所有非零可通行”处理。
YAstar 和 MinimumSnap 使用同一份私有快照：内切带置为障碍，未知区域按 `grid.allow_unknown` 决定，真实地图不被修改。
静态 ROS `OccupancyGrid` 仍遵循 `-1/0..100` 标准；只在静态输入和 RViz 可视化输出处转换。
MPC 对预测轨迹检查致命障碍和机器人半径、内切带中心以及未知区域策略；底层地图仍使用新编码。

这是原始地图消息编码的变更，必须一起重新编译并重启 map_server、动态障碍节点、planner、controller 和 nav_executor，不能混用旧进程或旧录制的 raw costmap 消息。

## 接口核查与使用范围

- YAstar 输入使用栅格单位的局部浮点坐标；wrapper 在边界统一转换回米，保留地图原点和半格中心语义，避免大世界坐标下的 float 精度损失。
- YAstar 的 `search()` 无解可能只返回起点；wrapper 验证实际到达目标栅格及整条路径的碰撞，不能用“非空”判成功。
- 同格起终点、地图边界、NaN/Inf、未知策略、取消请求均显式处理。搜索新增可选取消回调，原有双参数调用仍兼容；对角步不能切过障碍角。
- 每段路径先平滑再交给 topo 的 region / connector strip 检查。connector 等待点与出口强制精确到达，只有最终用户目标允许 `grid.tolerance`。
- MinimumSnap 源码已恢复为克隆仓库 HEAD 的原版，`src/MinimumSnapOsqp/` 无本地修改；wrapper 不改库内部求解条件、采样、碰撞迭代和求解器容差。
- 接入按 `tdt-nav-kit/main.cpp` 示例：order=6、maxDx=3、dt=0.1、TL=10、OSQPCorridor、collisionIteration=6、maxSpeed=3、maxAcc=1、corridorRange=2.5、corridorShrink=0、normTime=true、initVel=(0,0)。不强制 8/4 或库内严格碰撞模式；OSQPCorridor 的结果未通过检查时，使用 tdt 自带 Close 后端再次求解，保持阶数、时间分配、端点与速度约束。
- 前端用最大 0.1 m（至少半格）偏离的路径简化保留 YAstar 绕障转弯形状，并在视线安全、偏离小于阈值时清除局部多余控制点，避免 5 cm 栅格台阶被闭式平滑器插值成短波纹。起终点可安全直连时仅保留两个端点。平滑器收到零原点的米制路径与地图；地图额外补两格障碍边框以满足原版 SFC 的前提。原版 SFC 与碰撞检查对非零 origin 的符号不一致，因此不能传入负机器人原点。
- 原版求解分支每段固定采样 10 次，保留其原有 `setDt` 行为。输出采样折线每条边仍经过 wrapper 的 supercover 碰撞检查，包含对角触碰栅格；该检查不改变求解器本身。
- OSQP 失败后先重试原版 Close 平滑；两种 MinimumSnap 结果都不满足 success、端点和采样折线碰撞检查时，再尝试 wrapper 的 `safe_corner_cut` 几何备用。备用对控制点逐角切削两轮，每条新弦都通过同一 supercover 碰撞检查，最终以不大于 0.1 m 的间距采样并复检整条路径；无法安全切削的窄角保持原样。仅当备用确实圆滑了至少一个角且全路径安全时才接受，否则保留已验证的 YAstar 折线。不会把不安全曲线或未收敛的 OSQP 解当作成功。没有额外的控制点上限跳过条件；无安全搜索路径时抛异常。
- 日志 `MinimumSnap accepted: backend=close_retry controls=... samples=...` 表示实际采用了闭式求解的曲线，仍是 tdt MinimumSnap 的平滑结果。`backend=osqp_corridor` 表示首选走廊后端成功；`safe corner-cut backup accepted` 表示采用碰撞安全的几何备用；`MinimumSnap and safe corner-cut backup rejected` 表示最后回退 YAstar。`GridPlanner::lastSmoothingReport()` 提供同一验收状态用于回归。
- `smoothing_max_speed/acc` 只参与时间分配，不是严格动力学上限。输出仍是几何 `nav_msgs/Path`，没有逐点速度/时间协议，MPC 根据弧长、曲率和剩余制动距离生成速度参考。不同 topo 段之间不承诺导数连续。
- 平滑每轮开始/结束可取消；OSQP 单次线性求解不能由当前取消回调抢占，`smoothing_time_limit` 是库的迭代软限制。搜索循环可取消。
- 不接入 demo GUI、OpenCV 显示、kinodynamic A*、库里的临时掩码消息系统。动态障碍继续从本工程 map_server 进入快照。
- tdt 不直接消费原 costmap 的梯度惩罚；YAstar 使用占用信息及 `cost_field_radius` 生成自己的势场。MPC 目前仅使用障碍、内切带和未知区域的碰撞语义，不使用梯度值作优化代价。

## MPC 控制接口

`nav_executor` 接收几何 `nav_msgs/Path`；每个控制周期用最新 `map -> odom` TF 将路径变到 `odom`，用 `/Odometry` 的实际 `vx, vy, omega` 经精确时间戳 TF 变到 `base_link_fake`。二维位置和速度在 `odom` 中建模，离散间隔 0.05 s、预测 10 步，使用 qpOASES 解速度变量的凸二次规划；约束单轴速度与加速度，目标同时跟踪路径位置和按弧长生成的速度。求解后还会限制二维速度模长，并在局部代价地图上检查整条预测轨迹；失败则由执行器发零速并请求重新规划。命令经 `base_link_fake` 转换后发布到 `/cmd_vel`。当前 `fake_baselink` 不传递角速度，因此 MPC 输出 `angular.z=0`；`omega` 用于测量有效性检查，不参与本版二维平移模型。

控制参数在 `nav_executor.ros__parameters.MPC` 下，包括 `dt`、`horizon`、`max_speed`、`max_acc`、`max_lateral_acc`、`q_along`、`q_cross`、`r_speed`、`control_delay`、`max_working_set_recalculations`、`allow_unknown`。`allow_unknown` 要与规划器的 `TopoPlanner.grid.allow_unknown` 一致。qpOASES 源码和 LGPL-2.1 许可证位于 `nav_executor/third_party/qpOASES`。

当前配置保留了 `use_topology: false`，打开后需要有效的 `topology_yaml`。原来的 NavFn 源文件留存但不再参与编译，`grid.use_astar` 已移除。旧文档中的 `InnerPlanner` pluginlib 配置不适用于当前工程。

## 参数

在 `topology_planner.ros__parameters.TopoPlanner.grid` 下配置；已同步到 bringup 与 nav_executor 的 YAML。

|参数|默认值|单位/含义|
|---|---|---|
|tolerance|0.2|m，仅最终目标|
|allow_unknown|true|允许经过未知区域|
|use_final_approach_orientation|true|终点朝向沿路径切线，否则使用目标朝向|
|cost_field_radius|0.6|m，YAstar 势场半径|
|cost_weight|1.0|YAstar 搜索代价权重|
|smooth|true|启用 MinimumSnap|
|smoothing_backend|osqp_corridor|osqp_corridor / close|
|smoothing_dt|0.1|s，保留原版 setDt 行为|
|smoothing_max_speed|3.0|m/s，时间分配参数|
|smoothing_max_acc|1.0|m/s²，时间分配参数|
|smoothing_time_limit|10.0|s，迭代软限制|
|smoothing_iterations|6|最大迭代次数|
|smoothing_corridor_range|2.5|m，SFC 最大扩张范围|

区域约束、connector action/block_cmd、拓扑 YAML 中的 regions/connectors 格式继续由 topo 原有代码处理。平滑器不会改变单向边规则，也不会替代 region 检查。

## 构建与测试

默认直接编译 `/home/lyu/hxy/tdt-nav-kit` 中的三个算法源文件。可通过 `-DTDT_NAV_KIT_ROOT=/path/to/clone` 更改。构建需要 Eigen3、OpenCV core/imgproc、OSQP 与 OsqpEigen，tdt 源码使用 C++20，wrapper 对外仍为 C++17。

本机已复用现有构建的 OSQP / OsqpEigen 到 `tdt-nav-kit/3rd/install`；CMake 自动发现该目录，并将运行时依赖复制到 ROS 安装树的 lib 下。新机器可用 tdt 的 `scripts/setup.sh` 准备依赖，再构建。

```bash
cd /home/lyu/hxy/nav
source /opt/ros/jazzy/setup.bash
colcon build --packages-select map_server topology_global_planner nav_executor bringup --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select topology_global_planner nav_executor --event-handlers console_direct+
colcon test-result --verbose
```

回归覆盖地图编码、占用地图边界转换、膨胀方向、footprint 极值、平滑实际执行、米/栅格端点、障碍绕行、无解、越界、取消、未知区域和 connector 精确端点。转弯用例要求实际接受曲线、相邻线段航向变化小于 0.35rad，并独立检查所有曲线线段不穿墙。另覆盖反向、不同地图原点/分辨率、边缘扩张和无法平滑的窄拐角安全回退。

原先运行日志确认存在 OSQP 未收敛后连续回退的情况。本轮按用户要求恢复 tdt 原版源码与 main.cpp 示例条件，重编译后需重启导航进程再观察转弯效果。

当前回归中，OSQP 状态 7 后可由 Close 平滑产生可用的绕障曲线，不再直接退为 YAstar 折线。极窄拐角仍允许最终安全回退。完整仿真效果需重启导航后验证。
