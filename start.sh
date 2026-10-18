#!/usr/bin/env bash

set -eo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
LOG_DIR="$ROOT_DIR/log/launch_$(date +%Y%m%d_%H%M%S)"
pids=()

# Inherited by the launched processes: an orphaned backend keeps the lock held.
exec {launch_lock_fd}>"$ROOT_DIR/.simulation.lock"
if ! flock -n "$launch_lock_fd"; then
  echo "错误：已有仿真/导航在运行（可能只关闭了 UI）。请先停止旧启动进程。" >&2
  exit 1
fi
# Older launches predate this lock; detect their workspace-specific nodes too.
python3 - "$ROOT_DIR" <<'PY_CHECK'
import subprocess, sys
root = sys.argv[1]
leftovers = []
for row in subprocess.check_output(['ps', '-eo', 'pid=,comm=,args='], text=True).splitlines():
    fields = row.strip().split(None, 2)
    if len(fields) != 3: continue
    pid, comm, args = fields
    if comm in {'bash', 'sh', 'python3', 'ps', 'codex-linux-san'}: continue
    if root + '/nav/install/' in args or root + '/gazebo/install/' in args:
        leftovers.append((pid, comm))
if leftovers:
    print('错误：检测到当前工作区的后台节点尚未退出：' + ', '.join(
        name + ' (PID ' + pid + ')' for pid, name in leftovers), file=sys.stderr)
    raise SystemExit(1)
PY_CHECK

# 强制 Gazebo / RViz 使用 NVIDIA 独显（PRIME Render Offload）。
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export VK_DRIVER_FILES=/usr/share/vulkan/icd.d/nvidia_icd.json
export VK_ICD_FILENAMES="$VK_DRIVER_FILES"
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER MESA_LOADER_DRIVER_OVERRIDE

for setup_file in "$ROS_SETUP" "$ROOT_DIR/gazebo/install/setup.bash" "$ROOT_DIR/nav/install/setup.bash"; do
  if [[ ! -f "$setup_file" ]]; then
    echo "错误：找不到 $setup_file，请先安装 ROS 或编译对应工作空间。" >&2
    exit 1
  fi
done

mkdir -p "$LOG_DIR"

# 驱动或图形会话不可用时直接失败，确保不会静默使用软件渲染。
if ! command -v nvidia-smi >/dev/null 2>&1; then
  echo "错误：未找到 nvidia-smi，请安装并启用 NVIDIA 驱动。" >&2
  exit 1
fi
if ! nvidia-smi -L >"$LOG_DIR/nvidia.log" 2>&1; then
  echo "错误：NVIDIA 驱动不可用，无法强制使用独显。详情：$LOG_DIR/nvidia.log" >&2
  exit 1
fi
for driver_file in "$__EGL_VENDOR_LIBRARY_FILENAMES" "$VK_DRIVER_FILES"; do
  if [[ ! -r "$driver_file" ]]; then
    echo "错误：缺少 NVIDIA 图形驱动配置 $driver_file。" >&2
    exit 1
  fi
done
if [[ -n "${DISPLAY:-}" ]] && command -v glxinfo >/dev/null 2>&1; then
  if ! glxinfo -B >"$LOG_DIR/opengl.log" 2>&1 ||
      ! grep -qi 'OpenGL renderer string:.*NVIDIA' "$LOG_DIR/opengl.log"; then
    echo "错误：当前图形会话无法使用 NVIDIA OpenGL。详情：$LOG_DIR/opengl.log" >&2
    exit 1
  fi
fi
echo "已强制启用 NVIDIA GPU。驱动信息：$LOG_DIR/nvidia.log"

cleanup() {
  trap - INT TERM EXIT
  echo "停止仿真与导航..."
  # Gazebo server/GUI 会另建进程组，因此也记录本次启动的后代进程。
  descendants=()
  collect_children() {
    local child
    while read -r child; do
      [[ -n "$child" ]] || continue
      collect_children "$child"
      descendants+=("$child")
    done < <(pgrep -P "$1" || true)
  }
  for pid in "${pids[@]}"; do collect_children "$pid"; done
  for pid in "${pids[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  for ((attempt = 0; attempt < 10; attempt++)); do
    alive=0
    for pid in "${pids[@]}"; do
      if kill -0 -- "-$pid" 2>/dev/null; then alive=1; fi
    done
    if ((alive == 0)); then break; fi
    sleep 1
  done
  for pid in "${pids[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  for pid in "${descendants[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "${pids[@]}" "${descendants[@]}"; do
    kill -KILL "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_process() {
  local name="$1" workspace="$2"
  shift 2
  # 独立进程组便于退出时同时停止 launch 启动的所有子进程。
  setsid bash -c '
    set -e
    cd "$1"
    source "$2"
    source install/setup.bash
    shift 2
    exec "$@"
  ' bash "$ROOT_DIR/$workspace" "$ROS_SETUP" "$@" >"$LOG_DIR/$name.log" 2>&1 &
  pids+=("$!")
  echo "$name 已启动，日志：$LOG_DIR/$name.log"
}

start_process simulation gazebo ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
# 等待机器人落稳，避免 Point-LIO 在下落阶段初始化重力方向。
source "$ROS_SETUP"
python3 - <<'PY'
import time
import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
node = rclpy.create_node('wait_for_simulation')
ready = [False]
def clock_callback(msg):
    ready[0] = msg.clock.sec + msg.clock.nanosec * 1e-9 >= 1.5
subscription = node.create_subscription(Clock, '/clock', clock_callback, qos_profile_sensor_data)
deadline = time.monotonic() + 60
while not ready[0] and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
if not ready[0]:
    raise SystemExit('错误：60 秒内没有收到运行中的仿真时钟，请检查 simulation.log。')
PY
start_process bringup nav ros2 launch bringup startup.launch.py
start_process navigation nav ros2 launch nav_executor navigation_launch.py
start_process referee gazebo ros2 run referee_pub referee_sub.py

echo "启动流程已执行；按 Ctrl+C 停止全部进程。"
set +e
wait -n "${pids[@]}"
status=$?
set -e
echo "有启动进程退出（状态码 $status），请查看 $LOG_DIR 中的日志。" >&2
exit "$status"
