# Navigation executor

The active local controller is a generated Acados omnidirectional NMPC.

- Runtime tuning: [config/mpc.yaml](config/mpc.yaml)
- Offline generation settings: [config/solver.yaml](config/solver.yaml)
- Build, integration and validation: [docs/nmpc_implementation.md](docs/nmpc_implementation.md)
- Previous controller, excluded from the build: [legacy/qp_mpc](legacy/qp_mpc)

The standard launch files load the independent controller configuration through
`controller_params_file`. No legacy controller backend is linked or selectable.
