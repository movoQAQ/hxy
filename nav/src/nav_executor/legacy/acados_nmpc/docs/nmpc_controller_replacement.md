# MpcController 非线性 MPC 替换工程设计

版本：原始设计稿 v1，2026-10-18。当前已实施；以 [实施说明](nmpc_implementation.md) 为准，
其中记录归档策略、有限迭代结果验收、数值可行性修正等实施差异。

本文针对当前工作区源码，定义如何以 tud-amr/mpc_planner 为基础替换
`nav_executor::MpcController`。文中新增文件、接口、参数、命令均是实施约定，
不表示它们已经存在。本文为初始设计依据；已实施行为、构建与测试结果见 nmpc_implementation.md。

## 1. 范围与架构决策

目标是全向底盘的二维局部规划：输入几何路径、位姿、实测速度和局部地图，
输出平移速度 vx/vy。首版保持现有 ROS 话题、执行状态与任务代次机制。

采用以下固定决策，避免实施阶段在不同模型间混用：

- 物理位置 p=(x,y)，控制 u=(vx,vy)，全部在 odom 中优化。
- 内部增加速度记忆 m=(mx,my)，离散状态 z=(x,y,mx,my)。m 是上一控制，非实际速度状态。
- 控制周期与预测步长均为 0.05s，N=30，预测长度 1.5s。
- Acados 单求解器；首版不接 T-MPC++、概率约束或动态对象预测。
- 复用上游模型/模块组织、代码生成、求解器封装思路；新增全向 system 和模块。
- 不直接启用上游 Jackal ROS 节点；发布命令、停车和任务有效性仍由 PathExecutor 管理。
- 旧 QP 代码归档至 legacy/qp_mpc，不参与构建、链接和运行时选择。
- 优先完成位置/速度/平滑/避障；减速和方向权重初始为零，基础闭环稳定后逐项启用。

> 这是速度运动学模型，不是完整底盘动力学模型。惯性通过实测速度锚定、
> 加减速约束和延迟补偿近似。若实测跟踪滞后不可忽略，再单独评估实际速度状态模型。

## 2. 当前代码基线

以下路径相对 nav/：

| 文件 | 当前职责 | 实施动作 |
| --- | --- | --- |
| src/nav_executor/include/nav_executor/mpc_controller.hpp | initialize/setPlan/reset/compute 接口 | 保持原签名，增加诊断与已发布命令反馈 |
| src/nav_executor/src/mpc_controller.cpp | qpOASES、弧长参考、求解后碰撞检查 | 改为门面，旧实现移入独立后端 |
| src/nav_executor/src/path_executor.cpp | 输入时效、目标判断、发布、失败停车 | 适配末端控制与发布确认 |
| src/nav_executor/src/nav_executor_node.cpp | 任务代次和全局重规划 | 向监控传递局部规划反馈 |
| src/nav_executor/src/route_monitor.cpp | 偏离、停滞、路径阻塞检测 | 增加有限时效的绕行反馈 |
| src/map_server/include/map_server/costmap_client.hpp | 地图快照消费者 | 增加原子化读取地图和元数据的接口 |
| src/bringup/params/params.yaml | 当前启动默认配置 | 主配置迁移 |
| src/nav_executor/config/nav_executor.yaml | 包内备用配置 | 与主配置同步迁移 |
| src/navigation/fake_baselink/src/fake_baselink.cpp | fake→真实底盘速度旋转 | 保持当前输出契约 |

当前 MpcController 的限制：每轮构造新的 QP；保存的仅为上次命令，并非完整热启动；
障碍只用于求解后拒绝轨迹；各轴速度约束之后再做模长缩放。
新后端不得继承“求解后缩放即可满足全部约束”的假设。

### 2.1 输入输出契约

| 数据 | 坐标/时间/所有权 | 处理要求 |
| --- | --- | --- |
| nav_msgs/Path | map 或 header 指定坐标系，几何路径 | stamp 不当作到达时间；转换、去重、弧长化 |
| PoseStamped pose | local costmap global frame，当前为 odom | 与地图同系；检查有限值、TF 新鲜度 |
| Twist measured_speed | compute 接口收到时已在 base_link_fake 原点与坐标系 | 保留现有按 odom stamp 的 child→fake twist 转换；再旋转至 odom |
| 局部地图 | 自定义 Costmap2D，odom，9×9m，0.05m 分辨率 | 一次读取一致快照；优化与复核使用同一地图几何 |
| TwistStamped 返回值 | base_link_fake，当前时间 | linear.x/y 为平移速度，angular.z=0 |
| /cmd_vel | Twist，无 frame 字段 | PathExecutor 发布返回值的 twist；约定 fake 坐标系 |
| /cmd_vel_chassis | Twist，真实底盘坐标系 | fake_baselink 负责后续旋转；MPC 不重复处理 |

Point-LIO twist 使用 aft_mapped；当前 launch 将 Odometry remap 为 /odom。
不要用 odometry.header.frame_id 解释 twist。原点偏置的叉乘补偿由现有
transformOdometryVelocity 完成，不在新后端重复添加。

## 3. 目录与构建边界

建议工程结构：

```text
nav/src/
  mpc_planner/                         # 上游 fork，固定 commit
    solver_generator/                 # 增加离散动力学生成分支
    mpc_planner_modules/               # 全向目标和约束模块
    mpc_planner_omni/                   # 新 system，生成配置/脚本/manifest
      config/settings.yaml
      scripts/generate_omni_solver.py
      scripts/omni_model.py
      scripts/omni_objectives.py
      scripts/omni_constraints.py
      solver_manifest.json
  nav_executor/
    include/nav_executor/
      mpc_controller.hpp              # 兼容门面
      mpc/controller_backend.hpp
      mpc/types.hpp
      mpc/reference_builder.hpp
      mpc/obstacle_adapter.hpp
      mpc/warm_start.hpp
      mpc/trajectory_validator.hpp
      mpc/acados_backend.hpp
      mpc/legacy_qp_backend.hpp
    src/mpc_controller.cpp
    src/mpc/*.cpp
    test/test_mpc_reference.cpp
    test/test_mpc_obstacles.cpp
    test/test_mpc_warm_start.cpp
    test/test_nmpc_controller.cpp
    docs/nmpc_controller_replacement.md
```

生成产物由 mpc_planner_omni 导出一个 CMake target，例如 OmniMpc::solver；
nav_executor 仅经 AcadosBackend 调用它，不包含散落的生成文件路径。
上述 target 是新增约定，不是上游现成 target。

上游 C++ 模块与本地 adapter 之间只传值类型和几何参数，避免直接互传两种 Costmap2D 指针。
可视化和实验记录不是控制必需依赖，首版不纳入求解关键路径。

构建依赖必须锁定：mpc_planner commit、acados commit、CasADi 版本、
生成器配置哈希、目标架构、编译器。首次实现时填入真实版本，不能以 main/latest 发布。
保留 Apache-2.0 声明，并逐项登记实际引入依赖的许可。

## 4. 类接口与数据结构

### 4.1 兼容门面

现有四个方法保持签名：

```cpp
void initialize(const rclcpp::Node::SharedPtr & node,
  std::shared_ptr<tf2_ros::Buffer> tf, map_server::Costmap2D * costmap,
  std::string global_frame, std::string base_frame, double robot_radius);
void setPlan(const nav_msgs::msg::Path & path);
void reset();
geometry_msgs::msg::TwistStamped computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & measured_speed);
```

新增以下方法供 PathExecutor 使用，均为设计接口：

```cpp
void setCycleContext(CycleContext context); // 持有不可变地图快照、代次和时钟
const ControllerFeedback & feedback() const;
void notifyPublished(const PublishedCommand & command);
```

setCycleContext 在每次 compute 前调用；compute 消费后上下文失效，禁止复用旧上下文。
legacy 后端可忽略新增诊断字段，但不能绕过原有执行器检查。

### 4.2 关键值类型

```cpp
enum class SolveStatus {
  OK, INVALID_INPUT, NO_REFERENCE, OBSTACLE_OVERFLOW,
  INFEASIBLE, NUMERICAL_FAILURE, DEADLINE_EXCEEDED,
  COLLISION, OUTSIDE_MAP, STALE_INPUT
};

struct CycleContext {
  PlanIdentity identity;
  uint64_t control_sequence;
  uint64_t map_version;
  std::shared_ptr<const MapSnapshot> map;
  std::chrono::steady_clock::time_point began;
  std::chrono::steady_clock::time_point deadline;
};

struct ReferenceSample {
  Eigen::Vector2d position, tangent, normal;
  double arc, curvature, speed;
};

struct SolveRequest {
  CycleContext context;
  Eigen::Vector2d position_odom, measured_velocity_odom;
  Eigen::Vector2d last_published_velocity_odom;
  bool have_published_velocity;
  std::vector<ReferenceSample> reference; // N+1
  ObstacleSet obstacles;                 // 完整覆盖，固定槽位
};

struct SolveResult {
  SolveStatus status;
  std::vector<Eigen::Vector2d> positions; // N+1
  std::vector<Eigen::Vector2d> controls;  // N
  double solve_ms, max_constraint_violation;
};
```

ControllerFeedback 包含 identity、输入地图版本、生成时间、状态、路径弧长进度、
是否绕行、轨迹是否复核通过、最小间距和阶段耗时。PublishedCommand 必须记录
实际发布时刻、对应 solve sequence 和 odom 速度；停车命令同样进入历史。
这里的“发布”只证明 ROS 发布动作发生，不证明底盘执行，实际响应仍由 odom 校正。

### 4.3 后端职责

| 类 | 输入 | 输出/责任 |
| --- | --- | --- |
| ReferenceBuilder | 路径、当前投影、速度限制 | N+1 参考点与可制动速度曲线 |
| ObstacleAdapter | MapSnapshot、ROI、半径 | 保守障碍覆盖与覆盖完整性标记 |
| WarmStart | 已通过验证的旧解、当前状态 | 移位并重新 rollout 的初值 |
| AcadosBackend | SolveRequest、初值 | 原始解、原始求解状态与残差 |
| TrajectoryValidator | 原始解、地图、约束 | 发布前安全复核结果 |
| MpcController | 上述组件 | 返回 fake 坐标系首步命令与诊断 |

失败状态由门面映射为带稳定原因码的异常，保持当前 PathExecutor 的 fail 路径。
不要仅以英文自由文本区分错误；日志保留 reason_code 和上游原始 status。

## 5. 优化问题定义

### 5.1 变量与离散动力学

```text
z_k = [px_k, py_k, mx_k, my_k]   k=0..N
u_k = [vx_k, vy_k]               k=0..N-1
p_(k+1) = p_k + dt * u_k
m_(k+1) = u_k
```

m_0 取上次已发布 odom 速度；首次运行取当前实测速度。p_0 是延迟补偿后的当前位置。
所有内部向量必须使用同一个 odom 快照，不能保存 fake 坐标系历史后直接与 odom 控制相减。

将上游生成器扩展为显式分支：连续模型维持原流程；本模型使用
`model.disc_dyn_expr` 与 `integrator_type=DISCRETE`，不通过 RK4 近似 m 的更新。
Acados 支持该接口，但当前上游生成路径需要新增实现和生成测试。
禁止为迎合连续模型接口写成 dm/dt=u，这与 m_(k+1)=u_k 不等价。

### 5.2 参考速度和进度

1. setPlan 验证 frame、有限值与四元数，删除相邻重复点，建立弧长表。
2. 每轮使用单次 map→odom TF 转换路径，禁止逐点读取不同时间的 TF。
3. 在上一进度邻域投影；窗口按最大移动距离和恢复余量确定。交叉路径不得跳支。
4. 估计曲率。使用局部拟合时必须检查拟合曲线仍在可行域；退化为原折线时，
   将尖角作为需要降至低速/停车的事件，不能赋予虚假的有限转弯半径。
5. 生成 v_cap=min(v_max,sqrt(a_lat_max/(abs(kappa)+eps)))。
6. 末端速度设零，后向扫描 v_i<=sqrt(v_(i+1)^2+2*a_brake*ds)。
7. 前向扫描 v_(i+1)<=sqrt(v_i^2+2*a_accel*ds)，初值参考实测速度。
8. 以当前投影为起点按速度曲线采样 N+1 点；绕行/制动时冻结或放缓参考推进，
   下一轮仍从实测位置投影，不能无条件累加上一轮理想进度。

速度上限低于当前实测速度时，生成可实现的减速过程，不能要求首拍瞬时满足新上限。
实际状态已超安全包络时返回明确状态并停车，不能放宽碰撞约束以强求可行。
单点路径进入终点模式；不得以“路径无长度”为由反复全局重规划。

### 5.3 目标函数

取 rho(v)=sqrt(v·v+eps_v²)-eps_v，使零速为零且可微。
对于每一预测步：

```text
e = p_k - reference.position_k
J_path   = wc*(normal_k·e)^2 + wl*(tangent_k·e)^2
J_speed  = wv*(rho(u_k)-vref_k)^2
J_smooth = wdu*||u_k-m_k||²
J_brake  = wb*g_brake_k*positive_part(rho(m_k)-rho(u_k))²
J_dir    = wdir*g_dir_k*(1 - regularized_cos(u_k,m_k))
J_obs    = wo*sum_j positive_part(d_soft_j-d_j(p_k))²
J        = sum_k dt*J_k + J_terminal
```

positive_part 使用连续可微近似并统一 eps；不要对未正则化的 norm 在零点求导。
regularized_cos 的分母使用 sqrt(||u||²+eps²)*sqrt(||m||²+eps²)。
g_dir 在任一速度接近零时平滑衰减；低速区不要求保留旧方向。
g_brake 是本轮参考阶段参数，在必要减速、终点和避障制动阶段减小或置零。
方向与减速代价初始不启用，避免其非凸性和权重掩盖基础问题。

J_terminal 使用末端参考位置误差和 m_N 的零速误差。只有终点进入本轮可达视野时，
才将末端参考切换成最终目标并提高零速权重；远距离行驶不得每轮强制预测末端停车。
此外始终校验预测末端具有可行制动尾段，二者不是同一件事。

首版参考点作为逐步参数，不将 spline 进度作为决策状态。后续若引入 MPCC，
需要单独设计进度动力学，不能直接复用 unicycle 中 ds/dt=v 的假设。

### 5.4 硬约束

- ||u_k||² <= v_max²，必要时叠加底盘各轴限制。
- ||u_k-m_k||² <= (a_change_max*dt)²；a_change_max 为矢量变化上限。
- k=0 另约束 ||u_0-v_measured_at_actuation|| <= a_track_max*dt。
- 用正则化速度差限制加速/制动速率，参数 a_accel、a_brake 分开定义。
- 若实测速度与上次指令约束交集为空，报告 TRACKING_DIVERGENCE 原因；
  不静默删除实测约束。执行器停车并记录偏差，依据数据再决定是否调整模型。
- 每个障碍满足几何最小间距；地图可用范围按 footprint 收缩。
- 预测末端及其制动尾段均必须处于已观测地图的可行空间。
- 不以 slack 放松实际碰撞约束；只有舒适距离和软路径要求可软化。

延迟 tau_delay 首版以实测速度推进 p_0=p_now+tau_delay*v_measured。
独立复核必须覆盖 p_now→p_0 这段轨迹。延迟不是优化 dt，也不应每拍累计。
若底盘实际响应不符合此近似，应升级模型，不能靠加大热启动权重补偿。

## 6. 地图适配与安全复核

### 6.1 一致快照

为 CostmapClient 新增 readSnapshot()，返回地图数据、frame、stamp、接收墙钟、
current、radius/footprint、分辨率、origin 和本地单调递增 version。
读取与订阅写入受同一锁保护。当前地图锁能保护栅格复合读；新增版本与元数据
也必须纳入同一同步边界，不能在解锁后再拼接 snapshotStamp。
锁内只复制，几何构建和求解在锁外执行。

门面原 initialize 的裸地图指针保留给 legacy；NMPC 必须使用 CycleContext.map。
地图更新发生在求解中是正常情况：发布前若 version 已变化，使用最新快照重新做
轨迹/制动尾段碰撞检查；新图无效则拒绝发布。不能要求 version 永远不变，
也不能只检查 current=true 就忽略新障碍。

### 6.2 编码与障碍表示

原始编码固定为：0=致命障碍，1=未知，2=内切膨胀边界，255=自由。
3..254 是可通行等级，数值越大通常净空越大。未知单独按 allow_unknown 处理。

首版选用固定槽位的保守圆覆盖，max_obstacles=128 作为生成容量起点：

1. 从包含预测路径、绕行范围和制动尾段的 ROI 提取原始致命栅格；未知禁行时也纳入。
2. 单栅格可用中心加半对角线覆盖；合并时外接圆必须覆盖所有成员栅格的完整面积。
3. 使用自适应块划分合并，长墙不能用一个质心加任意小半径代替。
4. 障碍圆半径加机器人保守包络半径、padding、额外安全距离。
5. 每个圆提供 center/radius/active 参数；未使用槽位 active=0，用有限数值禁用约束。
6. 若容量不足，继续保守合并；若因此不可用或仍超容量，返回 OBSTACLE_OVERFLOW，
   停车/重规划。禁止只保留最近 128 个而没有剩余障碍安全证明。

圆覆盖可能保守地封死窄通道，这是首版已知限制。通过覆盖率和场景测试判断
是否需替换为凸多边形/走廊，不通过缩小障碍半径“解决”。
数值约束可使用 ||p-c||²-R²>=0；软距离用正则化 norm，避免中心处导数异常。
预测点之间仍可能穿过障碍，因此最终独立栅格扫掠检查不可省略。

机器人圆形包络与原地图膨胀只能计入一次：致命栅格做几何扩张，原膨胀等级仅用于
舒适距离参考；最终复核按原始致命/未知语义检查 footprint，并明确中心进入数值2
时的既有兼容策略。radius/padding 的来源必须与地图生产端一致，不能重复添加。

### 6.3 动态观测、拓扑与复核

DynamicObstacles 当前没有对象 ID、速度或未来轨迹。首版不宣称具备移动障碍预测。
可配置观测年龄裕量，但其保证取决于明确的障碍运动速度界；没有速度界时
不能宣称对任意移动障碍有安全保证。

最终复核至少包括：有限值、动力学残差、速度/变化率、障碍扫掠、地图边界、
延迟段、末端制动尾段；沿线采样步长不大于地图分辨率/2，并检查 footprint 覆盖栅格。
禁止复核后再做改变速度的缩放/滤波；若后处理改变控制，必须重新 rollout 与复核。

当前主配置 use_topology=false。启用拓扑时，局部避障不能跨越禁止区域：
必须由全局侧导出允许区域/连接走廊并加入约束。该接口完成前，拓扑模式下
不能承诺任意局部绕行；应限制到已验证走廊或关闭 NMPC 绕行功能。

## 7. 热启动与求解器生成

### 7.1 热启动规则

- 缓存上一轮已通过完整检查的解；只有 notifyPublished 后才更新上次指令历史。
- 正常周期前移一拍，尾部使用满足变化率的延拓；从新 p_0、m_0 重新 rollout。
- 调度间隔不是 50ms 时，按实际时间移位/插值，不能固定假设过去一拍。
- 新路径默认重置旧预测；首次初值从实测速度生成受限减速序列，不能瞬间设零。
- 初值碰撞不代表必然无解，但不能将旧初值当成可直接发布的 fallback。
- 定位跳变、ROS 时间回退、长控制间隔和 frame 变化时丢弃缓存。
- 原始/对偶变量是否同时热启动由固定 Acados 版本验证；不依赖未验证的自动行为。

上游 initializeWithBraking 和部分 visualize 直接读取 psi/v/a/w/spline。
这些调用需要全向版本或显式禁用；不能只换模型后继续调用原函数。
上游 ContouringModule 也读取这些字段，首版使用自定义 OmniTrackingModule。

### 7.2 生成器与产物契约

generate_omni_solver.py 的职责：

1. 检查版本锁与生成参数，建立 nx=4、nu=2 的 DISCRETE 模型。
2. 注册路径/速度/平滑/减速/方向/障碍目标及硬约束。
3. 对所有 stage 注册参数，区分初始、常规和末端约束；生成 C/C++ 与索引头。
4. 导出 N、dt、nx、nu、参数索引、约束索引和 max_obstacles。
5. 生成 manifest，写入配置哈希及依赖版本。
6. 执行离散状态更新、零速代价导数、参数维度和最小自由空间求解自检。

先使用有界迭代的 SQP 做正确性验证，再评估 SQP_RTI 性能。
RTI 返回成功不等于全部非线性约束已满足，所有模式均必须检查残差与轨迹。
采用适合自定义非线性代价的 Hessian 近似/正则化配置；不要把含方向项的
非凸目标直接当成天然半正定的最小二乘问题。

N/dt/nx/nu/槽位数变化必须重新生成，不能仅改 YAML。
权重、速度上限和障碍位置作为在线参数；参数修改仅在无活动路径时整批应用。
启动时校验 manifest 与 YAML，不一致即报错退出，不静默使用编译时旧数值。

## 8. 每周期处理和发布顺序

```text
PathExecutor tick:
  校验任务 identity、地图 current、TF 和 odom
  获取一致 MapSnapshot；记录 steady deadline 和 sequence
  读实测速度，完成 child→fake 转换
  controller.setCycleContext(...)
  controller.computeVelocityCommands(pose, speed)
    → 转换速度至 odom
    → ReferenceBuilder / ObstacleAdapter
    → 延迟补偿、构造初值、填参数、求解
    → 检查状态、残差、整条轨迹及制动尾段
    → 首步 odom→fake 旋转，返回命令
  复查任务有效性、输入时效、总耗时
  若地图已更新，用最新地图重检候选轨迹
  调用 monitor 反馈；该回调可能撤销路径
  再次确认 identity，发布 /cmd_vel
  controller.notifyPublished(...)
异常:
  先 publishZero，再 reset，沿原 failure 回调通知 TaskManager
```

发布零速度也要同步指令历史。compute 返回前不得把候选命令写成“已发布”。
reset 清除路径、求解缓存、进度和反馈；有新计划时首次以实测状态重新初始化。

首版保持同步求解，求解器实例由当前互斥回调独占。总周期目标不超过 50ms，
正常求解预算建议 25ms、发布截止建议 45ms，具体数值需目标硬件实测后确认。
限制迭代数并不能提供操作系统级硬超时；返回后超时检查只能拒绝迟到结果。
若不能满足预算，发布前必须完成工作线程/独立进程隔离及独立停车 watchdog，
不能仅增大 control_timeout 或让异步线程直接发布。

异步版必须绑定 identity/sequence/map_version，单槽 latest-only 请求队列，
由主执行器验证后发布，取消任务时丢弃旧结果；不能从其他线程修改正在求解的 capsule。

## 9. 执行器和路由监控的同步变更

### 9.1 终点与朝向

原逻辑在 distance<=xy_goal_tolerance 时立即零速，会绕开 NMPC 的末端控制。
改为：进入目标邻域仍允许 NMPC 制动；只有位置达标、实测平移速度达标，
且持续若干新鲜观测后才判成功并归零。建议连续3次有效观测，计数不能跨输入失效保留。
位置精度从场景要求配置，不默认沿用主配置的0.5m；初始可评估0.10m，非已保证精度。

本方案不控制 yaw。当前3.14并非严格“忽略朝向”，存在接近 pi 的边缘差异。
新增明确 goal_check_yaw=false；若要求终点朝向，需要另一个可执行旋转链路，
不能通过增大位置代价解决。成功判断也不应被自主旋转底盘的 angular.z 永久阻塞，
需明确平移任务是否检查角速度，默认本方案仅检查平移停止。

### 9.2 绕行反馈

RouteMonitor 新增 observeControllerFeedback。反馈必须满足同一 identity、
新鲜地图复核、短有效期（建议0.15s）和已发布控制对应 sequence。
仅凭求解成功不得持续豁免阻塞检查。

| 条件 | 处理 |
| --- | --- |
| 原全局线被占，但局部轨迹在允许走廊内安全且有进展 | 暂缓 PATH_BLOCKED，允许局部绕行 |
| 偏离原线，但仍在允许走廊内，且有有效绕行反馈 | 使用走廊边界判断，不机械触发0.5m阈值 |
| 反馈过期、走廊封闭、局部持续不可行 | 按现有事件触发停车与全局重规划 |
| MPC 持续输出低速/零速但目标未完成 | 独立等待超时；不能被现有低命令速度分支无限清零停滞计时 |
| 取消、新目标、plan_generation 变化 | 立即撤销旧反馈与候选解 |

绕行许可还需总时长上限和路径进度要求；建议初始最多2s无有效进展即重规划。
不能通过关闭全部 route_monitor 达成绕障。

## 10. 参数迁移

以下是待实现的配置示例，不可直接粘贴到当前版本期待生效：

```yaml
nav_executor:
  ros__parameters:
    controller_frequency: 20.0
    control_timeout: 0.045
    goal_check_yaw: false
    xy_goal_tolerance: 0.10
    stopped_velocity_tolerance: 0.05
    MPC:
      backend: acados              # legacy_qp | acados；仅启动时切换
      allow_unknown: true          # 沿用当前主配置；与上游策略一致
      dt: 0.05                     # 必须匹配生成 manifest
      horizon: 30                  # 必须匹配生成 manifest
      max_speed: 2.0
      max_acc: 2.0                 # 矢量速度变化上限
      max_accel: 2.0
      max_brake: 2.0
      max_tracking_acc: 2.0
      max_lateral_acc: 1.0
      control_delay: 0.05
      solver_budget_ms: 25.0
      safety_margin: 0.05
      max_obstacles: 128           # 必须匹配生成 manifest
      warm_start: true
      q_cross: 12.0
      q_along: 3.0
      r_speed: 1.5
      r_delta_velocity: 1.0
      r_brake: 0.0
      r_direction: 0.0
```

权重只是调试起点；因 dt 缩放、目标形式和正则化改变，不能保证与原QP相同响应。
旧 max_working_set_recalculations 只保留给 legacy，不映射成 Acados 迭代数。
停止速度、延迟、半径和容差必须只有一个明确来源，禁止后端各自读取不一致默认值。
若允许未知区域，独立复核也必须遵循相同策略；地图外仍不可当作未知可通行。

## 11. 实施批次与验收

| 批次 | 改动 | 通过条件 |
| --- | --- | --- |
| A | legacy 后端抽取、门面、值类型、地图快照、manifest校验 | 原有控制行为和TF测试保持通过；没有双发布者 |
| B | 离散模型、生成器、路径/速度/平滑、硬速度约束 | 纯求解测试验证模型更新、约束、零速导数 |
| C | 完整障碍覆盖、热启动、延迟段与制动尾段复核 | 障碍、边界、覆盖超容量与旧初值测试通过 |
| D | PathExecutor末端、发布确认、监控协作 | 到达/取消/重规划代次测试与闭环仿真通过 |
| E | 时间预算验证、参数扫描、逐项启用附加代价 | 目标硬件时延达标；停车和绕障指标无回退 |

必要测试矩阵：

| 场景 | 核查点 |
| --- | --- |
| odom/fake相差90°、map有旋转和平移 | 输出方向正确；串行转换仅做一次 |
| aft_mapped存在原点偏置与角速度 | 实测平移速度转换保持既有补偿 |
| 直线启动、已有运动速度接管 | 首步满足实测约束，无瞬时跳速 |
| 斜向高速 | 模长不超上限，无求解后裁剪 |
| 直角/急弯、平滑失败回退折线 | 提前制动，无伪低曲率导致高速切角 |
| 终点、单点路径、微小剩余距离 | 不除零、不反复重规划，位置和停止条件达标 |
| 横墙、长条障碍、窄门、稀疏动态点 | 覆盖完整，保守性可量化，不穿插值段 |
| ROI边界、未知区、地图外、障碍槽位溢出 | 策略一致且明确拒绝不完整模型 |
| 预测末端有墙但离散节点无碰撞 | 制动尾段检查拒绝危险解 |
| 局部绕行超过旧偏离阈值 | 走廊内可继续；走廊外或停滞能重规划 |
| 求解中地图更新/失效、TF失效 | 新图复核或停车，不发布过期控制 |
| 新目标、取消、重复交付旧generation | 不发布旧任务结果 |
| 超时、数值异常、无可行解 | 稳定错误码、零速发布、原failure链路生效 |
| ROS时间回退、控制间隔超长 | 丢弃热启动，按新实测状态恢复 |

记录指标：横向误差、终点误差、最小净空、速度跟踪误差、实际加减速、
方向变化、无必要减速次数、求解/全周期P50/P95/P99/最大时延、失败率、重规划次数。
正确性阈值按配置与数值容差检查；性能和闭环阈值在基线采集后固定。
不能仅以求解成功率或RViz轨迹看起来平滑作为验收依据。

实施后的命令模板（本次未执行；当前尚无新包/测试）：

```bash
# 在已配置ROS和锁定依赖的终端，工作目录为 nav/
python3 src/mpc_planner/mpc_planner_omni/scripts/generate_omni_solver.py
colcon build --packages-up-to nav_executor --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select nav_executor
colcon test-result --verbose
```

若第一阶段未改变 map_server 的公开API，上述构建仍应检查生成包是否被正确发现；
若增加 readSnapshot，则必须包含 map_server 的重建与消费者兼容验证。
回退通过启动配置 MPC.backend=legacy_qp 完成，不支持运行中热切换。

## 12. 已知边界与实施前必须完成的验证

- 上游网页源码已核查，但未在本机编译；必须先完成固定版本最小生成与C++链接。
- 上游原 Acados 封装有模型字段假设；DISCRETE分支和全向初始化均需真实实现。
- 地图只有当前障碍观测，不能验证未来动态对象轨迹避碰。
- 保守圆覆盖与128槽位是否能满足本地图的窄通道，需用真实地图快照验证。
- 同步模式无操作系统级硬实时保证；若实测预算不达标，线程/进程隔离是上线前置项。
- 底盘实际制动能力、命令失联停车机制与独立watchdog需要端到端确认。
- 1.5s预测域不能替代制动尾段验证；局部地图不足时应减速或停车。
- 拓扑允许走廊尚未作为当前Path接口的一部分传递，相关约束不能凭路径点自动推定。

## 13. 参考来源

上游参考使用 main 网页快照；实施时应替换为固定 commit 的永久链接。

- [mpc_planner说明与模块生成方式](https://github.com/tud-amr/mpc_planner)
- [上游动力学模型](https://github.com/tud-amr/mpc_planner/blob/main/solver_generator/solver_model.py)
- [上游Acados生成器](https://github.com/tud-amr/mpc_planner/blob/main/solver_generator/generate_acados_solver.py)
- [上游求解循环](https://github.com/tud-amr/mpc_planner/blob/main/mpc_planner/src/planner.cpp)
- [上游Contouring目标](https://github.com/tud-amr/mpc_planner/blob/main/mpc_planner_modules/scripts/contouring.py)
- [上游Acados热启动与刹车初始化](https://github.com/tud-amr/mpc_planner/blob/main/mpc_planner_solver/src/acados_solver_interface.cpp)
- [Acados Python接口：离散动力学、生成和求解选项](https://docs.acados.org/python_interface/index.html)

本地核查依据：mpc_controller.hpp/.cpp、path_executor.hpp/.cpp、route_monitor.hpp/.cpp、
nav_executor_node.cpp、两份导航YAML、navigation_launch.py、CostmapClient、Costmap/DynamicObstacles消息、
cost_values.hpp、grid_planner.cpp、fake_baselink.cpp、Point-LIO里程计发布及launch、SendToC_Board.py。
