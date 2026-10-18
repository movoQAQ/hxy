# Navigation executor

The active local controller is the restored qpOASES MPC, with actuator response,
curvature speed planning, lateral recovery and measured-speed goal confirmation.
Runtime parameters stay in [config/mpc.yaml](config/mpc.yaml), loaded by both launch
files through `controller_params_file`.

The Acados experiment and its integration/configuration snapshot are preserved in
[legacy/acados_nmpc](legacy/acados_nmpc). They are not built, linked, installed or
selectable. Navigation no longer needs Acados to build or run.

Goal completion requires position within 5 cm and measured speed below 0.03 m/s
for three distinct odometry samples. Path blockage/deviation/stall replanning,
odometry frame conversion and freshness checks are retained.

See [tuning and validation](docs/qp_tracking_tuning.md) for simulator parameters,
controller settings, regression commands and the scope of the validation.

## Navigation supervisor and motion FSM

The tactical tree now uses the `nav_task_interfaces/NavigateTask` action instead
of Nav2. TaskManager retains goal/plan identities; the independent MotionFSM owns
FOLLOW, HOLD, PREPARE_SPIN, SPIN, ESCAPE and HAZARD_RECOVERY. Recovery and spin
invalidate the previous route and require a fresh plan before tracking resumes.
See [interfaces, parameters and validation](docs/navigation_fsm.md).
