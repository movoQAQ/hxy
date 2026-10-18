#!/usr/bin/env python3
"""Discrete omnidirectional adaptation of tud-amr/mpc_planner's Acados generator.

Upstream: 3002da6a5e43b3577f584381bbcad569c82a381b (Apache-2.0).
See third_party/mpc_planner for the original generator and license.
Generate offline; the ROS executable never invokes Python or downloads dependencies.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

import casadi as cd
import numpy as np
import yaml
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

BASE = ["ref_x", "ref_y", "tangent_x", "tangent_y", "vref", "q_cross", "q_along",
        "r_speed", "r_delta", "r_brake", "r_direction", "q_obstacle", "terminal_weight",
        "terminal_stop", "measured_x", "measured_y", "track_active", "max_speed",
        "max_change", "max_accel", "max_brake", "max_tracking_acc", "min_x", "max_x",
        "min_y", "max_y", "soft_distance", "brake_gate"]
EPS = 1e-3


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, default=root/'config/solver.yaml')
    parser.add_argument('--output', type=Path, default=root/'generated')
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    n, dt, count = config['horizon'], config['dt'], config['max_obstacles']
    z = cd.SX.sym('z', 4)
    u = cd.SX.sym('u', 2)
    param = cd.SX.sym('p', len(BASE)+4*count)
    p = dict(zip(BASE, cd.vertsplit(param[:len(BASE)])))
    xy, memory = z[:2], z[2:]
    speed = lambda v: cd.sqrt(cd.dot(v, v)+EPS**2)-EPS
    positive = lambda v: (v+cd.sqrt(v*v+EPS**2))/2
    err = xy-cd.vertcat(p['ref_x'], p['ref_y'])
    tangent = cd.vertcat(p['tangent_x'], p['tangent_y'])
    normal = cd.vertcat(-p['tangent_y'], p['tangent_x'])
    tracking = p['q_cross']*cd.dot(normal, err)**2+p['q_along']*cd.dot(tangent, err)**2
    delta = u-memory
    delta_speed = speed(u)-speed(memory)
    cosine = cd.dot(u, memory)/(cd.sqrt(cd.dot(u, u)+EPS**2)*cd.sqrt(cd.dot(memory, memory)+EPS**2))
    gate = cd.dot(u,u)/(cd.dot(u,u)+0.05**2)*cd.dot(memory,memory)/(cd.dot(memory,memory)+0.05**2)
    stage = tracking+p['r_speed']*cd.sumsqr(u-p['vref']*tangent)+p['r_delta']*cd.dot(delta,delta)
    stage += p['r_brake']*p['brake_gate']*positive(-delta_speed)**2
    stage += p['r_direction']*gate*(1-cosine)
    geometry = [xy[0]-p['min_x'], p['max_x']-xy[0], xy[1]-p['min_y'], p['max_y']-xy[1]]
    soft = 0
    for j in range(count):
        cx, cy, radius, active = cd.vertsplit(param[len(BASE)+4*j:len(BASE)+4*(j+1)])
        d2 = cd.sumsqr(xy-cd.vertcat(cx,cy))
        geometry.append(active*(d2-radius**2)+(1-active))
        soft += active*positive(radius+p['soft_distance']-cd.sqrt(d2+EPS**2))**2
    tracking_velocity = u-cd.vertcat(p['measured_x'],p['measured_y'])
    bounds = [(p['max_speed']**2-cd.dot(u,u))/cd.fmax(p['max_speed']**2,1e-3),
              ((p['max_change']*dt)**2-cd.dot(delta,delta))/cd.fmax((p['max_change']*dt)**2,1e-4),
              cd.if_else(p['max_brake']<p['max_change']-1e-6,delta_speed+p['max_brake']*dt,1),
              cd.if_else(p['max_accel']<p['max_change']-1e-6,p['max_accel']*dt-delta_speed,1),
              p['track_active']*((p['max_tracking_acc']*dt)**2-cd.dot(tracking_velocity,tracking_velocity))/cd.fmax((p['max_tracking_acc']*dt)**2,1e-4)+(1-p['track_active'])]
    model = AcadosModel()
    model.name = 'omni_mpc'
    model.x, model.u, model.p = z, u, param
    model.disc_dyn_expr = cd.vertcat(xy+dt*u, u)
    model.cost_expr_ext_cost = stage+p['q_obstacle']*soft
    model.cost_expr_ext_cost_e = p['terminal_weight']*tracking+p['terminal_stop']*cd.dot(memory,memory)+p['q_obstacle']*soft
    model.con_h_expr = cd.vertcat(*(bounds+geometry))
    model.con_h_expr_0 = model.con_h_expr
    model.con_h_expr_e = cd.vertcat(*geometry)
    # Sanity check the model and cost derivatives at zero speed.
    f = cd.Function('step', [z,u], [model.disc_dyn_expr])
    assert np.allclose(np.array(f([1,2,0,0],[0.5,-0.5])).ravel(), [1+dt*.5,2-dt*.5,.5,-.5])
    deriv = cd.Function('deriv', [z,u,param], [cd.gradient(model.cost_expr_ext_cost,cd.vertcat(z,u))])
    assert np.isfinite(np.array(deriv(np.zeros(4),np.zeros(2),np.zeros(param.numel())))).all()
    ocp = AcadosOcp()
    ocp.model = model
    ocp.name = model.name
    ocp.solver_options.N_horizon = n
    ocp.solver_options.tf = n*dt
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.constraints.x0 = np.zeros(4)
    ocp.constraints.lh = np.zeros(len(bounds)+len(geometry))
    ocp.constraints.uh = np.full(len(bounds)+len(geometry),1e10)
    ocp.constraints.lh_0, ocp.constraints.uh_0 = ocp.constraints.lh.copy(), ocp.constraints.uh.copy()
    ocp.constraints.lh_e, ocp.constraints.uh_e = np.zeros(len(geometry)),np.full(len(geometry),1e10)
    ocp.parameter_values = np.zeros(param.numel())
    opt = ocp.solver_options
    opt.integrator_type = 'DISCRETE'
    opt.nlp_solver_type = 'SQP'
    opt.nlp_solver_max_iter = config['max_iterations']
    opt.tol = config['tolerance']
    opt.hessian_approx = 'EXACT'
    opt.regularize_method = 'MIRROR'
    opt.levenberg_marquardt = 1e-3
    opt.globalization = 'MERIT_BACKTRACKING'
    opt.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    opt.qp_solver_cond_N = n
    opt.qp_solver_iter_max = 60
    opt.qp_tol = 1e-6
    opt.print_level = 0
    opt.timeout_max_time = config['solver_budget_ms']/1000
    args.output.mkdir(parents=True, exist_ok=True)
    # Generate outside the checkout: no author initials or host paths in shipped sources.
    with tempfile.TemporaryDirectory(prefix='omni-codegen-') as scratch:
        ocp.code_gen_options.code_export_directory = scratch+'/code'
        ocp.code_gen_options.json_file = 'solver.json'
        AcadosOcpSolver.generate(ocp)
        expected = set()
        for src in Path(ocp.code_gen_options.code_export_directory).rglob('*'):
            if src.suffix not in ('.c','.h') or src.name.startswith('main_'):
                continue
            dst=args.output/src.relative_to(ocp.code_gen_options.code_export_directory)
            expected.add(dst)
            dst.parent.mkdir(parents=True,exist_ok=True)
            if not dst.exists() or dst.read_bytes() != src.read_bytes():
                shutil.copyfile(src,dst)
        for old in args.output.rglob('*'):
            if old.is_file() and old.suffix in ('.c','.h') and old not in expected:
                old.unlink()
    upstream='3002da6a5e43b3577f584381bbcad569c82a381b'
    acados_commit=subprocess.check_output(['git','-C',os.environ['ACADOS_SOURCE_DIR'],'rev-parse','HEAD'],text=True).strip()
    manifest=dict(model='omni_mpc',nx=4,nu=2,horizon=n,dt=dt,max_obstacles=count,
                  mpc_planner_commit=upstream,acados_commit=acados_commit,casadi=cd.__version__,
                  config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
                  generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    constants=['#pragma once','namespace nav_executor::mpc::generated {',
               f'inline constexpr int Horizon={n}, MaxObstacles={count}, ParameterCount={param.numel()};',
               f'inline constexpr double Dt={dt};',
               f'inline constexpr char AcadosCommit[]="{acados_commit}";',
               'enum Parameter {']
    constants += [f'  {name.upper()}={i},' for i,name in enumerate(BASE)]
    constants += [f'  OBSTACLES={len(BASE)}','};','}']
    (args.output/'solver_constants.hpp').write_text('\n'.join(constants)+'\n')
    print(json.dumps(manifest,indent=2))

if __name__ == '__main__':
    main()
