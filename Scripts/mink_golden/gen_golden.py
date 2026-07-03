# Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# --- LEGAL DISCLAIMER ---
# UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
# endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
# trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
#
# This plugin incorporates third-party software: MuJoCo (Apache 2.0),
# CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.
"""Golden-fixture generator: runs pinned Python mink and dumps JSON fixtures
that the URLabMink C++ automation tests replay. All inputs are stored in the
fixture; randomness only ever runs here, seeded."""

import argparse
import json
import math
import pathlib

import mink
import mujoco
import numpy as np
from mink.lie import SE3, SO3

HERE = pathlib.Path(__file__).parent
FIXTURES = HERE / "fixtures"
MODELS = HERE / "models"


def load_model(name: str) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(MODELS / f"{name}.xml"))


def _sanitize_inf(v):
    """json.dumps emits bare `Infinity`/`-Infinity` tokens for non-finite floats, which is not
    valid strict JSON and breaks FJsonSerializer::Deserialize on the whole file. Only the
    collision-avoidance limit's h vector ever contains real infinities (skipped rows); encode
    those as the strings "Infinity"/"-Infinity" instead — MinkJsonVec/MinkJsonMat on the C++ side
    special-case those two strings back to +/-infinity."""
    if isinstance(v, list):
        return [_sanitize_inf(e) for e in v]
    if isinstance(v, float) and math.isinf(v):
        return "Infinity" if v > 0 else "-Infinity"
    return v


def j(x):
    """numpy -> plain python for exact-round-trip json."""
    if isinstance(x, np.ndarray):
        return _sanitize_inf(x.tolist())
    if isinstance(x, (np.floating, np.integer)):
        return _sanitize_inf(x.item())
    return _sanitize_inf(x)


def sample_tangents(rng, dim):
    """Deterministic tangent set incl. edge cases (tiny angle, near pi)."""
    out = [np.zeros(dim)]
    for scale in (1e-12, 1e-7, 1e-3, 0.5, 1.5):
        out.append(scale * rng.standard_normal(dim))
    near_pi = rng.standard_normal(dim)
    near_pi *= (np.pi - 1e-6) / np.linalg.norm(near_pi[-3:] if dim == 6 else near_pi)
    out.append(near_pi)
    return out


def gen_lie():
    rng = np.random.default_rng(20260702)
    so3_tangents = sample_tangents(rng, 3)
    se3_tangents = sample_tangents(rng, 6)
    so3s = [SO3.exp(t) for t in so3_tangents] + [
        SO3(wxyz=q / np.linalg.norm(q))
        for q in [rng.standard_normal(4) for _ in range(4)]
    ]
    se3s = [SE3.exp(t) for t in se3_tangents] + [
        SE3.from_rotation_and_translation(r, rng.standard_normal(3)) for r in so3s[:4]
    ]
    v3s = [rng.standard_normal(3) for _ in range(3)]

    so3 = {
        "exp": [{"tangent": j(t), "wxyz": j(SO3.exp(t).wxyz)} for t in so3_tangents],
        "log": [{"wxyz": j(g.wxyz), "tangent": j(g.log())} for g in so3s],
        "matrix": [
            {"wxyz": j(g.wxyz), "matrix": j(g.as_matrix()),
             "wxyz_back": j(SO3.from_matrix(g.as_matrix()).wxyz)}
            for g in so3s
        ],
        "multiply": [
            {"a": j(a.wxyz), "b": j(b.wxyz), "out": j((a @ b).wxyz)}
            for a, b in zip(so3s, so3s[1:])
        ],
        "apply": [
            {"wxyz": j(g.wxyz), "v": j(v), "out": j(g.apply(v))}
            for g, v in zip(so3s, v3s * 3)
        ],
        "inverse": [{"wxyz": j(g.wxyz), "out": j(g.inverse().wxyz)} for g in so3s],
        "ljac": [{"tangent": j(t), "m": j(SO3.ljac(t))} for t in so3_tangents],
        "ljacinv": [{"tangent": j(t), "m": j(SO3.ljacinv(t))} for t in so3_tangents],
        "rpy": [{"wxyz": j(g.wxyz), "rpy": j(np.array(g.as_rpy_radians()))} for g in so3s],
        "rminus": [
            {"a": j(a.wxyz), "b": j(b.wxyz), "out": j(a.rminus(b))}
            for a, b in zip(so3s, so3s[1:])
        ],
        "interpolate": [
            {"a": j(a.wxyz), "b": j(b.wxyz), "alpha": 0.25,
             "out": j(a.interpolate(b, 0.25).wxyz)}
            for a, b in zip(so3s, so3s[1:])
        ],
    }
    se3 = {
        "exp": [{"tangent": j(t), "wxyz_xyz": j(SE3.exp(t).wxyz_xyz)} for t in se3_tangents],
        "log": [{"wxyz_xyz": j(g.wxyz_xyz), "tangent": j(g.log())} for g in se3s],
        "multiply": [
            {"a": j(a.wxyz_xyz), "b": j(b.wxyz_xyz), "out": j((a @ b).wxyz_xyz)}
            for a, b in zip(se3s, se3s[1:])
        ],
        "inverse": [{"wxyz_xyz": j(g.wxyz_xyz), "out": j(g.inverse().wxyz_xyz)} for g in se3s],
        "apply": [
            {"wxyz_xyz": j(g.wxyz_xyz), "v": j(v), "out": j(g.apply(v))}
            for g, v in zip(se3s, v3s * 4)
        ],
        "adjoint": [{"wxyz_xyz": j(g.wxyz_xyz), "m": j(g.adjoint())} for g in se3s],
        "ljac": [{"tangent": j(t), "m": j(SE3.ljac(t))} for t in se3_tangents],
        "ljacinv": [{"tangent": j(t), "m": j(SE3.ljacinv(t))} for t in se3_tangents],
        "jlog": [{"wxyz_xyz": j(g.wxyz_xyz), "m": j(g.jlog())} for g in se3s],
        "rminus": [
            {"a": j(a.wxyz_xyz), "b": j(b.wxyz_xyz), "out": j(a.rminus(b))}
            for a, b in zip(se3s, se3s[1:])
        ],
    }
    return {"so3": so3, "se3": se3}


def gen_configuration():
    rng = np.random.default_rng(20260703)
    result = {"models": {}}
    frames = {
        "arm3": [("ee", "site"), ("link2", "body"), ("g3", "geom")],
        "floating": [("tip", "site"), ("head", "body"), ("gb", "geom")],
    }
    for model_name, frame_list in frames.items():
        model = load_model(model_name)
        cfg = mink.Configuration(model)
        cases = []
        for i in range(4):
            q = model.qpos0.copy() + 0.3 * rng.standard_normal(model.nq)
            # Normalize free/ball quats so q is a valid configuration.
            for jnt in range(model.njnt):
                t, adr = model.jnt_type[jnt], model.jnt_qposadr[jnt]
                if t == mujoco.mjtJoint.mjJNT_FREE:
                    q[adr + 3 : adr + 7] /= np.linalg.norm(q[adr + 3 : adr + 7])
                elif t == mujoco.mjtJoint.mjJNT_BALL:
                    q[adr : adr + 4] /= np.linalg.norm(q[adr : adr + 4])
            cfg.update(q=q)
            v = rng.standard_normal(model.nv)
            case = {
                "q": j(q),
                "frames": [
                    {"name": n, "type": t,
                     "jac": j(cfg.get_frame_jacobian(n, t)),
                     "pose": j(cfg.get_transform_frame_to_world(n, t).wxyz_xyz)}
                    for n, t in frame_list
                ],
                "transform": {
                    "src": frame_list[0][0], "src_type": frame_list[0][1],
                    "dst": frame_list[1][0], "dst_type": frame_list[1][1],
                    "pose": j(cfg.get_transform(frame_list[0][0], frame_list[0][1],
                                                frame_list[1][0], frame_list[1][1]).wxyz_xyz)},
                "integrate": {"v": j(v), "dt": 0.02, "q_out": j(cfg.integrate(v, 0.02))},
                "inertia": j(cfg.get_inertia_matrix()),
            }
            cases.append(case)
        entry = {"cases": cases}
        if model.nkey:
            cfg.update_from_keyframe("home")
            entry["keyframe"] = {"name": "home", "q": j(cfg.q)}
        result["models"][model_name] = entry
    return result


def gen_utils():
    out = {"models": {}}
    for model_name in ("arm3", "floating"):
        model = load_model(model_name)
        q_ids, v_ids = mink.get_freejoint_dims(model)
        entry = {
            "freejoint_q_ids": q_ids, "freejoint_v_ids": v_ids,
            "subtree": [
                {"body": model.body(bid).name,
                 "body_ids": mink.get_subtree_body_ids(model, bid),
                 "geom_ids": mink.get_subtree_geom_ids(model, bid),
                 "joint_ids": mink.get_subtree_joint_ids(model, bid)}
                for bid in range(model.nbody)
            ],
        }
        if model_name == "floating":
            data = mujoco.MjData(model)
            mujoco.mj_kinematics(model, data)
            mink.move_mocap_to_frame(model, data, "mocap_target", "tip", "site")
            entry["move_mocap"] = {"mocap": "mocap_target", "frame": "tip", "type": "site",
                                   "pos": j(data.mocap_pos[0]), "quat": j(data.mocap_quat[0])}
        out["models"][model_name] = entry
    return out


def gen_qp():
    """Random strictly-convex QPs solved by the SAME backend family mink uses
    on the Python side (Goldfarb-Idnani via `quadprog`)."""
    import qpsolvers

    rng = np.random.default_rng(20260704)
    cases = []
    for n, n_ineq, n_eq in [(3, 0, 0), (5, 4, 0), (6, 8, 2), (9, 12, 3), (9, 0, 2)]:
        R = rng.standard_normal((n, n))
        H = R.T @ R + 1e-6 * np.eye(n)
        c = rng.standard_normal(n)
        G = rng.standard_normal((n_ineq, n)) if n_ineq else None
        h = (np.abs(rng.standard_normal(n_ineq)) + 0.1) if n_ineq else None
        A = rng.standard_normal((n_eq, n)) if n_eq else None
        b = 0.1 * rng.standard_normal(n_eq) if n_eq else None
        x = qpsolvers.solve_qp(H, c, G, h, A, b, solver="quadprog")
        assert x is not None
        # NOTE: the inequality bound is stored as "h_ineq", not "h" — Unreal's FJsonObject is
        # keyed by FString, whose GetTypeHash/operator== are case-insensitive, so a sibling "h"
        # field would collide with "H" (the Hessian) and silently clobber it on the C++ side.
        cases.append({"H": j(H), "c": j(c), "G": j(G) if G is not None else None,
                      "h_ineq": j(h) if h is not None else None,
                      "A": j(A) if A is not None else None,
                      "b": j(b) if b is not None else None, "x": j(x)})
    return {"cases": cases}


def _valid_q(rng, model):
    q = model.qpos0.copy() + 0.3 * rng.standard_normal(model.nq)
    for jnt in range(model.njnt):
        t, adr = model.jnt_type[jnt], model.jnt_qposadr[jnt]
        if t == mujoco.mjtJoint.mjJNT_FREE:
            q[adr + 3 : adr + 7] /= np.linalg.norm(q[adr + 3 : adr + 7])
        elif t == mujoco.mjtJoint.mjJNT_BALL:
            q[adr : adr + 4] /= np.linalg.norm(q[adr : adr + 4])
    return q


def _task_case(cfg, task):
    err = task.compute_error(cfg)
    jac = task.compute_jacobian(cfg)
    H, c = task.compute_qp_objective(cfg)
    res = task.compute_qp_residual(cfg)
    out = {"error": j(err), "jacobian": j(jac), "H": j(H), "c": j(c)}
    if res is not None:
        wj, we, mu = res
        out["residual"] = {"wjac": j(wj), "werr": j(we), "mu": j(mu)}
    return out


def gen_task_posture():
    rng = np.random.default_rng(20260705)
    out = {"models": {}}
    for model_name in ("arm3", "floating"):
        model = load_model(model_name)
        cfg = mink.Configuration(model)
        cases = []
        for cost, gain, lm in [(1.0, 1.0, 0.0), (0.5, 0.7, 0.1), ("vector", 1.0, 0.05)]:
            cost_vec = (np.abs(rng.standard_normal(model.nv)) if cost == "vector"
                        else np.array([cost]))
            task = mink.PostureTask(model, cost=cost_vec, gain=gain, lm_damping=lm)
            target = _valid_q(rng, model)
            task.set_target(target)
            q = _valid_q(rng, model)
            cfg.update(q=q)
            cases.append({"q": j(q), "cost": j(cost_vec), "gain": gain, "lm": lm,
                          "target_q": j(target), **_task_case(cfg, task)})
        out["models"][model_name] = cases
    return out


def gen_task_frame():
    rng = np.random.default_rng(20260706)
    out = {"models": {}}
    frames_by_model = {
        "arm3": [("ee", "site")],
        "floating": [("tip", "site"), ("head", "body")],
    }
    # (position_cost, orientation_cost, gain, lm_damping, use_config_target)
    variants = [
        (1.0, 1.0, 1.0, 0.0, False),
        ("vec3", "vec3", 1.0, 0.0, False),
        (1.0, 1.0, 1.0, 0.1, False),
        (1.0, 1.0, 0.5, 0.0, False),
        (1.0, 1.0, 1.0, 0.0, True),
    ]
    for model_name, frame_list in frames_by_model.items():
        model = load_model(model_name)
        cfg = mink.Configuration(model)
        cases = []
        for frame_name, frame_type in frame_list:
            for position_cost, orientation_cost, gain, lm, use_config_target in variants:
                pos_cost_vec = (np.abs(rng.standard_normal(3)) if position_cost == "vec3"
                                else np.array([position_cost]))
                ori_cost_vec = (np.abs(rng.standard_normal(3)) if orientation_cost == "vec3"
                                else np.array([orientation_cost]))
                task = mink.FrameTask(
                    frame_name=frame_name, frame_type=frame_type,
                    position_cost=pos_cost_vec, orientation_cost=ori_cost_vec,
                    gain=gain, lm_damping=lm,
                )
                if use_config_target:
                    target_q = _valid_q(rng, model)
                    cfg.update(q=target_q)
                    task.set_target_from_configuration(cfg)
                    target = task.transform_target_to_world
                else:
                    quat = rng.standard_normal(4)
                    quat /= np.linalg.norm(quat)
                    pos = rng.standard_normal(3)
                    target = SE3.from_rotation_and_translation(SO3(wxyz=quat), pos)
                    task.set_target(target)
                q = _valid_q(rng, model)
                cfg.update(q=q)
                cases.append({
                    "q": j(q), "frame": frame_name, "frame_type": frame_type,
                    "position_cost": j(pos_cost_vec), "orientation_cost": j(ori_cost_vec),
                    "gain": gain, "lm": lm, "target": j(target.wxyz_xyz),
                    **_task_case(cfg, task),
                })
        out["models"][model_name] = cases
    return out


def gen_task_relative_frame():
    rng = np.random.default_rng(20260707)
    out = {"models": {}}
    frame_root_by_model = {
        "arm3": ("ee", "site", "link1", "body"),
        "floating": ("tip", "site", "base", "body"),
    }
    # (position_cost, orientation_cost, gain, lm_damping, use_config_target)
    variants = [
        (1.0, 1.0, 1.0, 0.0, False),
        ("vec3", "vec3", 1.0, 0.0, False),
        (1.0, 1.0, 1.0, 0.1, False),
        (1.0, 1.0, 0.5, 0.0, False),
        (1.0, 1.0, 1.0, 0.0, True),
    ]
    for model_name, (frame_name, frame_type, root_name, root_type) in frame_root_by_model.items():
        model = load_model(model_name)
        cfg = mink.Configuration(model)
        cases = []
        for position_cost, orientation_cost, gain, lm, use_config_target in variants:
            pos_cost_vec = (np.abs(rng.standard_normal(3)) if position_cost == "vec3"
                            else np.array([position_cost]))
            ori_cost_vec = (np.abs(rng.standard_normal(3)) if orientation_cost == "vec3"
                            else np.array([orientation_cost]))
            task = mink.RelativeFrameTask(
                frame_name=frame_name, frame_type=frame_type,
                root_name=root_name, root_type=root_type,
                position_cost=pos_cost_vec, orientation_cost=ori_cost_vec,
                gain=gain, lm_damping=lm,
            )
            if use_config_target:
                target_q = _valid_q(rng, model)
                cfg.update(q=target_q)
                task.set_target_from_configuration(cfg)
                target = task.transform_target_to_root
            else:
                quat = rng.standard_normal(4)
                quat /= np.linalg.norm(quat)
                pos = rng.standard_normal(3)
                target = SE3.from_rotation_and_translation(SO3(wxyz=quat), pos)
                task.set_target(target)
            q = _valid_q(rng, model)
            cfg.update(q=q)
            cases.append({
                "q": j(q), "frame": frame_name, "frame_type": frame_type,
                "root": root_name, "root_type": root_type,
                "position_cost": j(pos_cost_vec), "orientation_cost": j(ori_cost_vec),
                "gain": gain, "lm": lm, "target": j(target.wxyz_xyz),
                **_task_case(cfg, task),
            })
        out["models"][model_name] = cases
    return out


def gen_task_com():
    rng = np.random.default_rng(20260708)
    out = {"models": {}}
    # (cost, gain, lm_damping, use_config_target)
    variants = [
        (1.0, 1.0, 0.0, False),
        ("vec3", 1.0, 0.0, False),
        (1.0, 1.0, 0.1, False),
        (1.0, 0.5, 0.0, False),
        (1.0, 1.0, 0.0, True),
    ]
    for model_name in ("arm3", "floating"):
        model = load_model(model_name)
        cfg = mink.Configuration(model)
        cases = []
        for cost, gain, lm, use_config_target in variants:
            cost_vec = (np.abs(rng.standard_normal(3)) if cost == "vec3"
                        else np.array([cost]))
            task = mink.ComTask(cost=cost_vec, gain=gain, lm_damping=lm)
            if use_config_target:
                target_q = _valid_q(rng, model)
                cfg.update(q=target_q)
                task.set_target_from_configuration(cfg)
                target = task.target_com
            else:
                target = rng.standard_normal(3)
                task.set_target(target)
            q = _valid_q(rng, model)
            cfg.update(q=q)
            cases.append({
                "q": j(q), "cost": j(cost_vec), "gain": gain, "lm": lm,
                "target": j(target),
                **_task_case(cfg, task),
            })
        out["models"][model_name] = cases
    return out


def gen_task_damping():
    rng = np.random.default_rng(20260709)
    out = {"models": {}}
    for model_name in ("arm3", "floating"):
        model = load_model(model_name)
        cfg = mink.Configuration(model)
        cases = []
        for cost in (1.0, "vector"):
            cost_vec = (np.abs(rng.standard_normal(model.nv)) if cost == "vector"
                        else np.array([cost]))
            task = mink.DampingTask(model, cost_vec)
            q = _valid_q(rng, model)
            cfg.update(q=q)
            cases.append({"q": j(q), "cost": j(cost_vec), **_task_case(cfg, task)})
        out["models"][model_name] = cases
    return out


def gen_task_dof_freezing():
    rng = np.random.default_rng(20260710)
    out = {"models": {}}
    dof_indices_by_model = {"arm3": [0, 2], "floating": [0, 1, 7]}
    for model_name, dof_indices in dof_indices_by_model.items():
        model = load_model(model_name)
        cfg = mink.Configuration(model)
        cases = []
        for gain in (1.0, 0.6):
            task = mink.DofFreezingTask(model, dof_indices=dof_indices, gain=gain)
            q = _valid_q(rng, model)
            cfg.update(q=q)
            cases.append({
                "q": j(q), "dof_indices": dof_indices, "gain": gain,
                **_task_case(cfg, task),
            })
        out["models"][model_name] = cases
    return out


def gen_task_kinetic_energy():
    rng = np.random.default_rng(20260711)
    out = {"models": {}}
    for model_name in ("arm3", "floating"):
        model = load_model(model_name)
        cfg = mink.Configuration(model)
        task = mink.KineticEnergyRegularizationTask(cost=1e-4)
        task.set_dt(0.02)
        cases = []
        for _ in range(2):
            q = _valid_q(rng, model)
            cfg.update(q=q)
            H, c = task.compute_qp_objective(cfg)
            cases.append({"q": j(q), "cost": 1e-4, "dt": 0.02, "H": j(H), "c": j(c)})
        out["models"][model_name] = cases
    return out


def gen_task_equality():
    rng = np.random.default_rng(20260712)
    out = {"models": {}}
    model = load_model("equality")
    cfg = mink.Configuration(model)
    # (equality_names, cost)
    variants = [
        ([], [1.0]),
        (["eq_connect"], [1.0]),
        ([], [1.0, 0.5]),
    ]
    cases = []
    for equality_names, cost in variants:
        cost_vec = np.array(cost)
        task = mink.EqualityConstraintTask(
            model, cost=cost_vec,
            equalities=(equality_names or None),
        )
        for _ in range(3):
            q = _valid_q(rng, model)
            cfg.update(q=q)
            cases.append({
                "q": j(q), "equality_names": equality_names, "cost": j(cost_vec),
                **_task_case(cfg, task),
            })
    out["models"]["equality"] = cases
    return out


def gen_limit_configuration():
    rng = np.random.default_rng(20260713)
    out = {"models": {}}
    # (gain, min_distance_from_limits)
    variants = [(0.95, 0.0), (0.5, 0.05)]
    for model_name in ("arm3", "floating"):
        model = load_model(model_name)
        cfg = mink.Configuration(model)
        cases = []
        for gain, min_distance_from_limits in variants:
            limit = mink.ConfigurationLimit(
                model, gain=gain, min_distance_from_limits=min_distance_from_limits
            )
            for _ in range(3):
                q = _valid_q(rng, model)
                cfg.update(q=q)
                constraint = limit.compute_qp_inequalities(cfg, dt=0.0)
                cases.append({
                    "gain": gain,
                    "min_distance_from_limits": min_distance_from_limits,
                    "q": j(q),
                    "G": j(constraint.G) if constraint.G is not None else None,
                    "h": j(constraint.h) if constraint.h is not None else None,
                })
        out["models"][model_name] = cases
    return out


def gen_limit_velocity():
    rng = np.random.default_rng(20260714)
    out = {"models": {}}
    # (velocities dict (insertion order matters), dt)
    velocities_by_model = {
        "arm3": {"j1": 3.14, "j2": 2.0, "j3": 2.0},
        "floating": {"neck": [1, 1, 1], "ant": 2.0},
    }
    dts = [0.02, 0.005]
    for model_name, velocities in velocities_by_model.items():
        model = load_model(model_name)
        cfg = mink.Configuration(model)
        # Store the joint order explicitly as a list of [name, [values...]] pairs so the C++
        # test rebuilds the exact same ordered TArray<TPair<FString, FMinkVec>> (JSON objects
        # preserve insertion order in Python, but we want this to be unambiguous on the C++
        # side too).
        joint_order = [[name, list(np.atleast_1d(vel).astype(float))] for name, vel in velocities.items()]
        limit = mink.VelocityLimit(model, velocities=velocities)
        cases = []
        for dt in dts:
            for _ in range(3):
                q = _valid_q(rng, model)
                cfg.update(q=q)
                constraint = limit.compute_qp_inequalities(cfg, dt=dt)
                cases.append({
                    "joint_order": j(joint_order),
                    "dt": dt,
                    "q": j(q),
                    "G": j(constraint.G) if constraint.G is not None else None,
                    "h": j(constraint.h) if constraint.h is not None else None,
                })
        out["models"][model_name] = cases
    return out


def gen_limit_collision():
    model_name = "scene_collision"
    model = load_model(model_name)
    cfg = mink.Configuration(model)

    # Single geom_pairs spec exercised across every case below: g1/g2/g3 vs the two static
    # obstacles and the floor, plus a self-collision pair (g1 vs g3). Filtering (welded/
    # parent-child/contype-conaffinity) is entirely mink's own _construct_geom_id_pairs; g1 ends
    # up excluded from every obstacle/floor pair here because link1's parent IS the worldbody
    # that owns floor/obstacle1/obstacle2 (parent-child check), leaving 7 surviving pairs.
    geom_pairs = [(["g1", "g2", "g3"], ["obstacle1", "obstacle2", "floor"]), (["g1"], ["g3"])]

    # Four q's, found by grid-searching mj_geomDistance over this exact model (all within each
    # joint's own range) to geometrically exercise distinct regimes:
    #   home:                 the arm3 "home" keyframe (elbow/wrist already penetrate the floor).
    #   stretch_obstacle1:    g2 brought within ~4cm of obstacle1 (sweet spot for the
    #                         detection_dist=0.05 variant, out of range for the default 0.01).
    #   near_floor:           g3 brought within ~2cm of the floor, not penetrating.
    #   near_self_collision:  j2/j3 folded to their range limits so g1-g3 (self collision) closes
    #                         to ~1.2cm, while g2/g3 deeply penetrate the floor as a bonus case.
    q_cases = [
        ("home", [0.0, 0.4, -0.8]),
        ("stretch_obstacle1", [-0.1, -1.2, -0.2]),
        ("near_floor", [-0.3, -0.5, 1.6666666666666667]),
        ("near_self_collision", [0.0, 1.9, 2.4]),
    ]

    # (label, gain, minimum_distance_from_collisions, collision_detection_distance,
    #  bound_relaxation) — each variant overrides exactly one field from the defaults.
    param_variants = [
        ("defaults", 0.85, 0.005, 0.01, 0.0),
        ("gain_0.5", 0.5, 0.005, 0.01, 0.0),
        ("min_dist_0.01", 0.85, 0.01, 0.01, 0.0),
        ("detection_0.05", 0.85, 0.005, 0.05, 0.0),
        ("bound_relaxation_0.01", 0.85, 0.005, 0.01, 0.01),
    ]

    dt = 0.02
    cases = []
    for q_label, q_list in q_cases:
        q = np.array(q_list)
        cfg.update(q=q)
        for variant_label, gain, min_dist, detection, relax in param_variants:
            limit = mink.CollisionAvoidanceLimit(
                model,
                geom_pairs,
                gain=gain,
                minimum_distance_from_collisions=min_dist,
                collision_detection_distance=detection,
                bound_relaxation=relax,
            )
            # Deviation #5: sort the deduped geom-id pairs ascending so the C++ port (which
            # sorts identically in its ctor) produces rows in the same order.
            limit.geom_id_pairs.sort()
            limit.max_num_contacts = len(limit.geom_id_pairs)
            constraint = limit.compute_qp_inequalities(cfg, dt=dt)
            cases.append({
                "q_label": q_label,
                "variant": variant_label,
                "q": j(q),
                "gain": gain,
                "min_dist": min_dist,
                "detection_dist": detection,
                "bound_relaxation": relax,
                "dt": dt,
                "geom_id_pairs": [[int(a), int(b)] for a, b in limit.geom_id_pairs],
                "G": j(constraint.G),
                "h": j(constraint.h),
            })

    return {"models": {model_name: cases}, "geom_pairs": geom_pairs}


def gen_solve_ik():
    rng = np.random.default_rng(20260706)
    scenarios = []

    def run(model_name, q0, tasks_desc, limits_mode, constraints_desc, damping, dt, steps=10,
            limits_desc=None):
        model = load_model(model_name)
        cfg = mink.Configuration(model)
        cfg.update(q=q0)
        tasks, t_json = [], []
        for d in tasks_desc:
            if d["type"] == "frame":
                t = mink.FrameTask(d["frame"], d["frame_type"], d["position_cost"],
                                   d["orientation_cost"], d.get("gain", 1.0), d.get("lm", 0.0))
                t.set_target(mink.SE3(wxyz_xyz=np.array(d["target"])))
            elif d["type"] == "posture":
                t = mink.PostureTask(model, d["cost"], d.get("gain", 1.0), d.get("lm", 0.0))
                t.set_target(np.array(d["target_q"]))
            elif d["type"] == "damping":
                t = mink.DampingTask(model, d["cost"])
            elif d["type"] == "com":
                t = mink.ComTask(d["cost"]); t.set_target(np.array(d["target_com"]))
            tasks.append(t); t_json.append(d)
        limits = None if limits_mode == "default" else ([] if limits_mode == "none" else limits_mode)
        constraints = None
        c_json = None
        if constraints_desc:
            constraints = [mink.DofFreezingTask(model, d["dofs"]) for d in constraints_desc]
            c_json = constraints_desc
        v = mink.solve_ik(cfg, tasks, dt, solver="quadprog", damping=damping,
                          limits=limits, constraints=constraints)
        q = q0.copy()
        traj_cfg = mink.Configuration(model); traj_cfg.update(q=q0)
        for _ in range(steps):
            vi = mink.solve_ik(traj_cfg, tasks, dt, solver="quadprog", damping=damping,
                               limits=limits, constraints=constraints)
            traj_cfg.integrate_inplace(vi, dt)
        # "limits" is normally the plain string mode ("default"/"none"); when limits_mode is an
        # explicit list of Limit objects (as for the collision-avoidance scenario), the caller
        # passes limits_desc — a JSON-serializable dict describing exactly how to reconstruct
        # that same explicit list on the C++ side — which is stored instead of the string.
        if limits_desc is not None:
            limits_json = limits_desc
        else:
            limits_json = limits_mode if isinstance(limits_mode, str) else "default"
        scenarios.append({"model": model_name, "q0": j(q0), "tasks": t_json,
                          "limits": limits_json,
                          "constraints": c_json, "damping": damping, "dt": dt,
                          "v": j(v), "steps": steps, "q_final": j(traj_cfg.q)})

    arm = load_model("arm3")
    home = arm.key_qpos[0].copy()
    ee_target = j(mink.SE3.from_rotation_and_translation(
        mink.SO3.identity(), np.array([0.45, 0.1, 0.35])).wxyz_xyz)
    run("arm3", home,
        [{"type": "frame", "frame": "ee", "frame_type": "site", "position_cost": 1.0,
          "orientation_cost": 0.2, "lm": 0.01, "target": ee_target},
         {"type": "posture", "cost": 1e-2, "target_q": j(home)}],
        "default", None, 1e-12, 0.02)
    run("arm3", home,
        [{"type": "frame", "frame": "ee", "frame_type": "site", "position_cost": 1.0,
          "orientation_cost": 0.0, "lm": 0.1, "target": ee_target},
         {"type": "damping", "cost": 1e-1}],
        "none", [{"dofs": [0]}], 1e-3, 0.02)
    flt = load_model("floating")
    q0 = flt.qpos0.copy()
    run("floating", q0,
        [{"type": "frame", "frame": "tip", "frame_type": "site", "position_cost": 1.0,
          "orientation_cost": 0.5, "target": j(mink.SE3.from_rotation_and_translation(
              mink.SO3.identity(), np.array([0.3, 0.2, 1.5])).wxyz_xyz)},
         {"type": "posture", "cost": 1e-3, "target_q": j(q0)}],
        "default", None, 1e-12, 0.01)

    # Collision-avoidance scenario: an explicit [ConfigurationLimit, CollisionAvoidanceLimit]
    # list (not the "default"/"none" shorthand), driving the ee toward obstacle1 so the
    # collision limit actually engages during the integration loop.
    scene = load_model("scene_collision")
    scene_home = scene.key_qpos[0].copy()
    config_limit = mink.ConfigurationLimit(scene, gain=0.95)
    collision_geom_pairs = [(["g1", "g2", "g3"], ["obstacle1", "obstacle2", "floor"]), (["g1"], ["g3"])]
    collision_gain, collision_min_dist = 0.5, 0.01
    collision_detection, collision_relax = 0.05, 0.01
    collision_limit = mink.CollisionAvoidanceLimit(
        scene, collision_geom_pairs, gain=collision_gain,
        minimum_distance_from_collisions=collision_min_dist,
        collision_detection_distance=collision_detection,
        bound_relaxation=collision_relax,
    )
    collision_limit.geom_id_pairs.sort()
    collision_limit.max_num_contacts = len(collision_limit.geom_id_pairs)
    scene_ee_target = j(mink.SE3.from_rotation_and_translation(
        mink.SO3.identity(), np.array([0.35, 0.1, 0.3])).wxyz_xyz)
    run("scene_collision", scene_home,
        [{"type": "frame", "frame": "ee", "frame_type": "site", "position_cost": 1.0,
          "orientation_cost": 0.1, "lm": 0.01, "target": scene_ee_target},
         {"type": "posture", "cost": 1e-3, "target_q": j(scene_home)}],
        [config_limit, collision_limit], None, 1e-12, 0.02,
        limits_desc={
            "configuration": {"gain": 0.95},
            "collision": {
                "gain": collision_gain,
                "min_dist": collision_min_dist,
                "detection_dist": collision_detection,
                "bound_relaxation": collision_relax,
                "geom_pairs": collision_geom_pairs,
                "geom_id_pairs": [[int(a), int(b)] for a, b in collision_limit.geom_id_pairs],
            },
        })

    return {"scenarios": scenarios}


LAYERS = {
    "lie": gen_lie,
    "configuration": gen_configuration,
    "utils": gen_utils,
    "qp": gen_qp,
    "task_posture": gen_task_posture,
    "task_frame": gen_task_frame,
    "task_relative_frame": gen_task_relative_frame,
    "task_com": gen_task_com,
    "task_damping": gen_task_damping,
    "task_dof_freezing": gen_task_dof_freezing,
    "task_kinetic_energy": gen_task_kinetic_energy,
    "task_equality": gen_task_equality,
    "limit_configuration": gen_limit_configuration,
    "limit_velocity": gen_limit_velocity,
    "limit_collision": gen_limit_collision,
    "solve_ik": gen_solve_ik,
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--layers", default=",".join(LAYERS))
    args = parser.parse_args()
    FIXTURES.mkdir(exist_ok=True)
    for layer in args.layers.split(","):
        data = LAYERS[layer]()
        path = FIXTURES / f"{layer}.json"
        path.write_text(json.dumps(data))
        print(f"wrote {path} ({path.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
