# Source and modifications

Source: https://github.com/tud-amr/mpc_planner
Commit: 3002da6a5e43b3577f584381bbcad569c82a381b
License: Apache-2.0, preserved in LICENSE.

`generate_acados_solver.upstream.py` is an unmodified reference snapshot of
`solver_generator/generate_acados_solver.py`. It is not executed or compiled.
The active generator `scripts/generate_omni_solver.py` adapts its Acados model,
OCP, constraints, cost and C generation flow: explicit discrete omni dynamics,
previous-command memory, independent terminal constraints, six custom objectives,
bounded SQP, generated parameter indices and a reproducible manifest.
The upstream Jackal runtime and its model-dependent braking/visualization are
not connected. ROS integration stays in nav_executor.
