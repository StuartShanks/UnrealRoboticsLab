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


def j(x):
    """numpy -> plain python for exact-round-trip json."""
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    return x


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


LAYERS = {
    "lie": gen_lie,
    # Later tasks register: configuration, utils, qp, task_*, limit_*, solve_ik
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
