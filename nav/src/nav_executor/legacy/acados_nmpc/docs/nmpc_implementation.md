# Acados 全向 NMPC：实施与使用

## 接入状态

当前唯一活动控制器为 `MpcController` 中的 Acados NMPC。原 QP 头文件、实现、
测试及 qpOASES 全部保留在 `legacy/qp_mpc/`，没有加入 CMake target，未链接至导航节点，
也没有运行时后端选择开关。

新实现基于 tud-amr/mpc_planner 的 Acados 生成流程定制，并非直接运行其 Jackal 节点。
原生成器快照、来源提交和 Apache-2.0 许可位于 `third_party/mpc_planner/`。
实际生成脚本为 `scripts/generate_omni_solver.py`；无需整套上游 ROS 包即可构建本地适配。

## 参数入口

- `config/mpc.yaml`：在线运行参数的唯一控制器配置，包括目标判断、代价、速度和安全限制。
- `config/solver.yaml`：离线生成维度、迭代上限、默认求解预算和求解精度。
- `generated/manifest.json`：生成时的版本、参数文件及生成脚本 SHA256。
- `config/nav_executor.yaml`：仍保留任务/全局规划备用配置，不再嵌入 MPC 参数。
- `bringup/params/params.yaml`：仍保留地图、任务和全局规划配置，不再嵌入 MPC 参数。

`navigation_launch.py` 与 `bringup_launch.py` 新增 `controller_params_file` 参数，默认读取
本包安装目录的 `config/mpc.yaml`。参数覆盖顺序为：导航配置、控制器配置、仿真时钟、
路径参数。通过这两种 launch 启动会自动加载新控制器参数。

```bash
# 在 nav 工作区，已 source ROS 与 install/setup.bash
ros2 launch nav_executor navigation_launch.py
# 自定义运行配置
ros2 launch nav_executor navigation_launch.py controller_params_file:=/absolute/path/mpc.yaml
```

MPC.* 参数为启动时参数，运行中的 ROS 参数修改会明确拒绝，避免参数服务器显示新值而
求解器仍使用旧值。修改 horizon/dt/max_obstacles 必须重新生成；启动时会验证其与编译
常量一致。控制频率必须等于1/dt，当前固定20Hz。

## 模型与代价

```text
z = [x, y, previous_vx, previous_vy]，u = [vx, vy]
p_(k+1) = p_k + 0.05*u_k
previous_velocity_(k+1) = u_k
N=30，预测1.5秒
```

previous_velocity 是指令记忆，不是底盘实际速度状态。每拍通过里程计实测速度再次约束
首步，并保留延迟段检查。首步同时受已发布指令和实测速度限制，交集为空时报告
TRACKING_DIVERGENCE 并停车。

实现六项代价：横向/纵向路径误差、目标速度、控制变化、障碍缓冲距离、减速、速度方向变化。
为改善零速附近求解条件，目标速度使用 `||u-v_ref*tangent||²`，而不是非凸的速度模长
匹配；路径代价和有限权重仍允许绕行。减速/方向代价使用正则化速度，低速关闭方向惩罚，
必要减速阶段关闭减速惩罚。两项附加权重默认0，基础闭环稳定后可在配置中逐项调大。

速度曲线由弧长、曲率限速、终点零速及前后向加减速扫描生成；路径时间戳不用于推导速度。
尖角按需停车处理。优化模型为 DISCRETE，未以RK4近似上一指令的记忆更新。

硬限制包括速度模长、速度矢量变化、可配置加速/制动、首步实测跟踪、地图边界和障碍。
当标量加减速限制被更严格的矢量变化限制完全蕴含时，NLP中禁用重复约束；C++仍逐项复核。
不对最终发布命令单独裁剪或滤波。有限SQP迭代如有微小速度边界误差，可做至多0.005m/s的
逐步可行性修正，再重新积分整条轨迹，逐项复核全部速度限制、几何障碍、地图扫掠和制动尾段。
超过此修正幅度或任一检查失败即拒绝；反馈restored记录是否发生修正。

## 地图、安全与失败

CostmapClient.readSnapshot() 在同一锁内复制地图、时间、接收墙钟、current和版本。
求解器使用自定义栅格语义：0障碍、1未知、2膨胀边界、255自由。
当前只提供障碍快照，没有动态对象速度和未来轨迹预测。

ObstacleAdapter 采用保守分块外接圆，完整覆盖障碍栅格，不截断最近若干障碍。
容量不足时扩大合并块；覆盖过于保守可能导致通道不可行，此时停车/重规划，不能缩小障碍。
机器人包络取配置 footprint 的最大半径，已含地图端 padding，再加独立 safety_margin。
对原始致命栅格扩张一次，不把整片膨胀层当成新的致命几何重复扩张。

发布前独立校验速度、变化率、动力学、完整扫掠、延迟段和预测末端制动尾段。
沿线采样间隔不大于半个栅格，并增加半采样间隔的半径裕量以覆盖采样间隙。
地图外不可通行；未知区域按统一 allow_unknown 处理。求解期间地图更新时，用最新快照
再次复核，地图失效则停车。

仅 Acados 的 SUCCESS、MAXITER、TIMEOUT 状态可进入候选复核。后两者仅表示目标未完全
收敛的有界迭代，不能直接发布：必须在真实求解时间预算内，且所有独立物理约束、
几何障碍约束、动力学和安全扫掠全部通过。反馈记录原始solver_status与converged。
`nav_executor/controller_status`发布JSON诊断，包含任务代次、原始求解状态、converged、restored、
绕行标记、地图版本与耗时；停车时valid=false。
数值异常、QP失败、不可行、超实际预算或任何复核失败均沿现有failure回调先发布零速，
再撤销路径并交给任务管理器恢复。

SQP最多20次迭代，内部软时间预算取配置预算的80%，为末次迭代和调度保留余量。
25ms为求解实际时间预算，45ms为求解加复核发布预算；这是有界迭代和迟到结果拒绝，
不等于操作系统硬实时保证。底盘失联停车、响应延迟和实际制动能力仍需实机确认。

## 执行与重规划

- 保持 `/cmd_vel` 的 base_link_fake 速度语义，fake_baselink继续旋转到真实底盘。
- 历史指令仅在 PathExecutor 真正发布后更新；候选控制不会提前写成已发布状态。
- 终点需要位置和实测平移停止条件，并由3个不同时间戳的有效里程计样本确认。
- 当前任务只控制平移，默认不要求yaw或角速度归零。
- 新目标、取消和任务代次失效继续使用原有TaskManager确认机制。
- 最近0.15秒内有已发布、安全复核通过的局部轨迹时，暂缓对原全局线的阻塞重规划；
  停滞检测仍独立工作，低速或零指令也不能无限重置停滞计时。
- 只有实际绕行反馈才暂缓原路径偏离检测；整条预测轨迹仍需处于max_deviation路径管内。
- 当前没有拓扑区域走廊接口。拓扑模式需要显式设置MPC.allow_detours=false，
  此时局部路径管收紧至0.05m；这不是正式的拓扑多边形约束，尚不能宣称严格区域保证。

## 构建与再生成

当前锁定：

- mpc_planner：3002da6a5e43b3577f584381bbcad569c82a381b
- acados：8ff7c2a17e235d6cad6671538e923ccb7428dc3e
- CasADi：3.7.2
- 官方模板渲染器：0.2.1，下载后校验SHA256

Linux x86_64的依赖配方：

```bash
# 工作目录 nav；首次需要网络、编译器、CMake和python3-venv
bash src/nav_executor/scripts/setup_acados.sh
# 默认使用当前CPU支持的BLASFEO目标和AVX HPIPM；部署到不同CPU必须重建运行库
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to nav_executor --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select nav_executor
colcon test-result --verbose
```

已提交生成的C源码，普通构建不调用Python或访问网络。首次仅准备运行库也可采用匹配提交的
外部Acados安装，配置 `-DACADOS_ROOT=/path/to/install`，并提供lib/git_commit_hash标记。
CMake验证提交；安装时携带匹配运行库和许可，库使用ORIGIN相对查找依赖。

需要重新生成时：

```bash
ACADOS_SOURCE_DIR="$PWD/.cache/acados-source" \
  .cache/nmpc-venv/bin/python src/nav_executor/scripts/generate_omni_solver.py
```

生成在临时目录进行，仅复制C/H及不含主机绝对路径的manifest，避免把作者工作目录写进源码。
所有新增命名使用omni/mpc/nmpc等中性名称。

## 验证记录

2026-10-18 本机验证：

- `colcon build --packages-up-to nav_executor`：map_server、topology_global_planner、nav_executor均通过。
- 新增控制器/几何/监控测试19项全部通过，包括理想闭环终点制动与静态障碍绕行。
- `colcon test-result`：nav_executor汇总26项、topology_global_planner汇总13项，均无错误、失败或跳过。
- 已安装launch参数检查通过，独立controller_params_file入口生效。
- 隔离ROS域174启动/退出正常，日志确认活动后端为Acados omni NMPC，N=30、dt=0.05。
- 隔离ROS域175使用真实launch、合成地图/TF/里程计与理想底盘完成目标任务：
  初始(2,2)，目标(3,2)，结束(3.0017,2.0000)，29条已复核命令，
  最大求解20.0141ms、最大compute总耗时20.1180ms；使用的原始状态为0和7，
  其中12条指令做了受限可行性修正，完整安全复核均通过。
- 主控制库符号扫描无qpOASES/QProblem；生成manifest与脚本/参数SHA256一致。
- 本包源码、配置、生成代码和文档命名扫描不含作者简称。

完整ROS理想闭环可复现命令（显式隔离域，避免给正常导航域发布模拟数据）：

```bash
ROS_DOMAIN_ID=175 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 src/nav_executor/test/run_ros_closed_loop.py
```

以上为本机单次结果，不是最坏时延证明。尚未运行完整Gazebo动力学回归或实机测试；
真实速度滞后、轮胎侧滑、制动距离与移动障碍行为仍需在对应环境验证。
