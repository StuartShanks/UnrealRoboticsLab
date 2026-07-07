"""Headless mobile_tidybot.py: deterministic target script, per-step trace dump.

Replicates mink v1.2.0 examples/mobile_tidybot.py exactly (fix_base=False,
no pause), minus the viewer/keyboard. See docs/superpowers/plans/
2026-07-07-tidybot-mink-ik.md for the target script + schema.
"""
import json
from pathlib import Path

import mujoco
import numpy as np

import mink

_HERE = Path(__file__).parent
_XML = _HERE / "models" / "stanford_tidybot" / "scene.xml"
_OUT = _HERE / "fixtures" / "tidybot_trace.json"

N_STEPS = 2500
INTEGRATE_DT = 0.005  # the example's rate.dt at 200 Hz
SOLVER = "daqp"       # adjust per Step 2 if unavailable
POS_THRESHOLD = 1e-4
ORI_THRESHOLD = 1e-4
MAX_ITERS = 20

JOINT_NAMES = ["joint_x", "joint_y", "joint_th"] + [f"joint_{i}" for i in range(1, 8)]


def target_pos(k: int, p0: np.ndarray) -> np.ndarray:
    reach = np.array([0.0, 0.35, 0.15])
    far = np.array([0.9, 0.0, 0.0])
    if k < 300:
        return p0
    if k < 400:
        return p0 + (k - 300) / 100.0 * reach
    if k < 900:
        return p0 + reach
    if k < 1000:
        s = (k - 900) / 100.0
        return p0 + (1 - s) * reach + s * far
    theta = 2.0 * np.pi * (k - 1000) / 1200.0
    return p0 + far + np.array([0.4 * (np.cos(theta) - 1.0), 0.4 * np.sin(theta), 0.0])


def main() -> None:
    model = mujoco.MjModel.from_xml_path(_XML.as_posix())
    data = mujoco.MjData(model)

    dof_ids = np.array([model.joint(name).id for name in JOINT_NAMES])
    actuator_ids = np.array([model.actuator(name).id for name in JOINT_NAMES])

    configuration = mink.Configuration(model)

    end_effector_task = mink.FrameTask(
        frame_name="pinch_site", frame_type="site",
        position_cost=1.0, orientation_cost=1.0, lm_damping=1.0,
    )
    posture_cost = np.zeros((model.nv,))
    posture_cost[3:] = 1e-3
    posture_task = mink.PostureTask(model, cost=posture_cost)
    tasks = [end_effector_task, posture_task]
    limits = [mink.ConfigurationLimit(model)]

    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    configuration.update(data.qpos)
    posture_task.set_target_from_configuration(configuration)
    mujoco.mj_forward(model, data)
    mink.move_mocap_to_frame(model, data, "pinch_site_target", "pinch_site", "site")

    site_id = model.site("pinch_site").id
    mocap_id = model.body("pinch_site_target").mocapid[0]
    p0 = data.mocap_pos[mocap_id].copy()
    q0 = data.mocap_quat[mocap_id].copy()  # wxyz

    steps = []
    for k in range(N_STEPS):
        data.mocap_pos[mocap_id] = target_pos(k, p0)
        data.mocap_quat[mocap_id] = q0
        T_wt = mink.SE3.from_mocap_name(model, data, "pinch_site_target")
        end_effector_task.set_target(T_wt)

        rec = {"q_in": configuration.q.tolist(),
               "target_pos": data.mocap_pos[mocap_id].tolist(),
               "target_quat": q0.tolist()}

        n_iters = 0
        v0 = None
        for i in range(MAX_ITERS):
            vel = mink.solve_ik(configuration, tasks, INTEGRATE_DT, SOLVER, damping=1e-3)
            if i == 0:
                v0 = vel.copy()
            configuration.integrate_inplace(vel, INTEGRATE_DT)
            n_iters = i + 1
            err = end_effector_task.compute_error(configuration)
            if (np.linalg.norm(err[:3]) <= POS_THRESHOLD
                    and np.linalg.norm(err[3:]) <= ORI_THRESHOLD):
                break

        data.ctrl[actuator_ids] = configuration.q[dof_ids]
        mujoco.mj_step(model, data)

        assert np.all(np.isfinite(data.qpos)), f"NaN qpos at step {k}"
        assert np.all(np.isfinite(data.qacc)), f"NaN qacc at step {k}"

        rec.update({
            "v0": v0.tolist(),
            "q_out": configuration.q.tolist(),
            "n_iters": n_iters,
            "ctrl": data.ctrl[actuator_ids].tolist(),
            "q_sim": data.qpos.tolist(),
            "ee_sim_pos": data.site_xpos[site_id].tolist(),
            "ee_sim_quat": _site_quat(data, site_id),
        })
        steps.append(rec)

    out = {"meta": {"mink": mink.__version__ if hasattr(mink, "__version__") else "1.2.0",
                    "mujoco": mujoco.__version__, "model": "stanford_tidybot/scene.xml",
                    "n_steps": N_STEPS, "integrate_dt": INTEGRATE_DT,
                    "sim_timestep": model.opt.timestep, "solver": SOLVER,
                    "joint_names": JOINT_NAMES},
           "steps": steps}
    _OUT.write_text(json.dumps(out))
    print(f"wrote {_OUT} ({len(steps)} steps, {_OUT.stat().st_size/1e6:.1f} MB)")


def _site_quat(data, site_id):
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, data.site_xmat[site_id])
    return quat.tolist()


if __name__ == "__main__":
    main()
