# hxy

ROS 2 Jazzy 仿真、导航及地图编辑工作空间。

## 目录

| 目录 / 文件 | 内容 |
| --- | --- |
| `gazebo/` | 仿真场景、机器人模型及启动文件 |
| `nav/` | 定位、代价地图、规划、MPC、导航 FSM 和战术行为树 |
| `map_edit/` | 地图编辑功能包 |
| `tdt-nav-kit/` | T-DT 导航工具源码 |
| `nav-10-17日晚上备份/` | 旧版导航工作空间的源码备份 |
| `start.sh` | 仿真与导航的一键启动入口，强制使用 NVIDIA GPU |
| `edit.sh` | 地图编辑启动入口 |

`nav/src/localization/topology_global_planner` 为仓库中的普通 ROS 功能包，
与导航源码一起维护，无需初始化该包的 Git 子模块。

## 启动与构建

在已配置 ROS 2 Jazzy、依赖及 NVIDIA 驱动的环境中：

```bash
./start.sh --help
```

各工作空间按其自身的说明使用 `colcon build --symlink-install` 编译。
导航 FSM 的接口、参数和使用方式见
[navigation_fsm.md](nav/src/nav_executor/docs/navigation_fsm.md)。

## 上传范围

保留源码、地图、模型、配置、启动脚本、文档及旧工作空间源码备份。
编译产物、安装目录、日志、缓存以及嵌套仓库的 Git 元数据保留在本机，不纳入版本控制。

导入前各仓库的本地 Git 历史保存在 `.repository-history/`，未删除。
原项目许可证及版权声明随源文件保留。
