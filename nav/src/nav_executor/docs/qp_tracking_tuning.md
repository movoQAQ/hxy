# QP MPC tracking and stopping

## Problems addressed

The controller previously advanced its reference despite lateral error and used
an immediate velocity response. A sharp bend could move the reference away while
the real steering mechanism was still turning. Progress now permits limited
backtracking, lateral error reduces reference speed, and a bounded lateral
correction brings the vehicle back toward the path.

The previous 0.5 m arrival tolerance could end execution while the vehicle was
still moving. Completion now requires both position and measured speed to pass;
passing the goal outside those conditions leaves the controller active, including
reverse correction after an overshoot.

## Actual simulated actuators

The installed robot model uses the source model in
`pb2025_robot_description/resource/xmacro/pb2025_sentry_robot.sdf.xmacro`.
Its SwerveDrive configuration overrides the plugin defaults:

| Setting | Value |
| --- | ---: |
| Wheel radius | 0.0758 m |
| Wheel angular acceleration | 50 rad/s² |
| Steering proportional gain | 6 s⁻¹ |
| Steering angular velocity | 3 rad/s |
| Steering angular acceleration | 10 rad/s² |
| Wheelbase / track | 0.5 / 0.5 m |
| Tire friction μ / μ₂ | 1 / 1 |

The wheel rolling acceleration limit is theoretically 3.79 m/s². Body motion also
depends on steering lag, contact and slip, so this is not used as the controller's
guaranteed acceleration. Simulator wheel parameters are unchanged.

## Controller changes and parameters

All active controller tuning remains in `config/mpc.yaml`.

| Parameter | Value | Purpose |
| --- | ---: | --- |
| `dt`, `horizon` | 0.05 s, 30 | 1.5 s prediction |
| `max_speed` | 1.4 m/s | Conservative cruise limit |
| `max_acc`, `max_brake` | 0.8 m/s² | Command slew and braking profile |
| `max_lateral_acc` | 0.6 m/s² | Curvature speed limit |
| `max_direction_rate` | 1.2 rad/s | First-command direction continuity |
| `response_time`, `control_delay` | 0.2 s, 0.08 s | Approximate actuator response and delay |
| `curvature_window` | 0.2 m | Curvature sampling distance |
| `q_along`, `q_cross` | 5, 20 | Emphasize lateral path recovery |
| `recovery_distance` | 0.3 m | Scale reference speed with lateral error |
| `recovery_gain`, `max_recovery_speed` | 1.5 s⁻¹, 0.6 m/s | Bounded lateral correction |
| `projection_backtrack` | 0.5 m | Allow recovery after advancing past a segment |
| `goal_speed_gain` | 1.2 s⁻¹ | Reduce speed close to path end |
| `terminal_weight` | 5 | Emphasize predicted final position |
| `r_speed`, `r_delta_velocity` | 1.5, 0.3 | Velocity tracking and command continuity |
| `max_working_set_recalculations` | 500 | qpOASES iteration budget |

Prediction starts from measured velocity and uses an exact discrete first-order
velocity response. The 0.2 s response constant is a validated approximation, not
a measured parameter from the wheel plugin. Curvature limits, a backward braking
pass, preview and distance-based final slowdown form the speed profile. Velocity
and command acceleration constraints use inscribed octagons inside the QP.
Collision checks include the predicted response and a braking tail.

Arrival requires distance ≤ 0.05 m and measured planar speed ≤ 0.03 m/s for three
distinct odometry stamps. Heading and angular-speed checks remain disabled for
the current position navigation interface. The command validity timeout is 0.05 s.

Both navigation parameter files set `TopoPlanner.grid.tolerance: 0.0`: the global
planner must reach the requested endpoint, rather than silently replacing it with
a nearby point up to 0.2 m away. An unreachable requested endpoint now fails
planning. Existing path blockage, deviation and stall replanning still apply.

## Validation

Gazebo tests use the installed robot and actual wheel plugin on isolated flat
ground, with a free costmap and ground-truth odometry. Each case uses a new model;
they do not modify or command the user's running arena. Results are recorded in
[qp_tracking_validation.json](qp_tracking_validation.json).

| Case | Result |
| --- | --- |
| 4 m straight | 4.30 cm final error; no overshoot |
| Radius 0.65 m hairpin | 1.63 cm maximum lateral error; 0.67 m/s maximum turn speed |
| Initial 0.4 m lateral error | Recovered within 8 cm in 1.35 s |
| Sudden nearby goal while moving | Overshot 0.41 m, reversed and settled within 2.04 cm |

The last case deliberately supplies a target inside the stopping distance. Its
overshoot demonstrates continued correction, not a guarantee of instantaneous
stopping. Maximum observed solve time across these runs was 8.26 ms.

The real navigation executor also passed an isolated ROS integration test with a
0.25 s response plant: final error 3.81 cm, measured speed at success 0.0197 m/s.
The test observed entry into the goal region while still moving before success.
Controller unit tests additionally cover velocity limits, collision rejection,
lagged stopping, lateral recovery, hairpins, overshoot and moving handoff.

From a workspace with ROS Jazzy, simulator and navigation environments sourced:

```bash
# From the repository root, parent of nav/ and gazebo/:
python3 nav/src/nav_executor/test/run_swerve_tracking.py
# From nav/:
ROS_DOMAIN_ID=188 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 src/nav_executor/test/run_qp_executor_closed_loop.py
ROS_DOMAIN_ID=187 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  ctest --test-dir build/nav_executor --output-on-failure
```

These tests do not cover the full arena, localization noise or all surface
conditions. Restart navigation to load the rebuilt controller and parameters,
then send a new goal. The simulator itself does not need restarting.
