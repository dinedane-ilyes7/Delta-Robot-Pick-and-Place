"""Load the MJCF generated from the CAD and check it behaves like the delta of robot.yaml.

  python3 cad/validate_mujoco.py
1. Loop closure: the 6 connect constraints are satisfied at the reference pose.
2. Hold: PD on the three motors keeps the platform at the reference pose under gravity.
3. Move: drive the arms to the IK angles of other points; the platform must land on them
   (checks the hinge axes, ball centres and rod lengths of the model against the kinematics).
"""
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "analysis"))
from kinematics import Delta  # noqa: E402
import yaml  # noqa: E402

CFG = yaml.safe_load(open(os.path.join(ROOT, "config", "robot.yaml")))
G = CFG["geometry"]
DELTA = Delta(G["R"], G["r"], G["L1"], G["L2"])
XML = os.path.join(HERE, "out", "mujoco", "delta_cell.xml")


def connect_residual(m, d):
    rows = [i for i in range(d.nefc) if d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_EQUALITY]
    return float(np.max(np.abs(d.efc_pos[rows]))) if rows else 0.0


def run(m, d, target_fn, T, kp=300.0, kd=1.0):
    jid = [m.joint(f"j{n}").qposadr[0] for n in (1, 2, 3)]
    vid = [m.joint(f"j{n}").dofadr[0] for n in (1, 2, 3)]
    gear = m.actuator_gear[:3, 0]
    worst = 0.0
    for _ in range(int(T / m.opt.timestep)):
        q = d.qpos[jid]
        qd = d.qvel[vid]
        tau = kp * (target_fn(d.time) - q) - kd * qd
        d.ctrl[:3] = np.clip(tau / gear, m.actuator_ctrlrange[:3, 0], m.actuator_ctrlrange[:3, 1])
        mujoco.mj_step(m, d)
        worst = max(worst, connect_residual(m, d))
    return worst


def main():
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    out = {}
    tcp = m.site("platform_tcp").id
    p_ref = d.xpos[m.body("platform").id].copy()
    out["1_closure_residual_at_reference_mm"] = connect_residual(m, d) * 1000
    th_ref = np.array([d.qpos[m.joint(f"j{n}").qposadr[0]] for n in (1, 2, 3)])
    out["reference_platform_mm"] = (p_ref * 1000).round(3).tolist()
    out["reference_theta_deg"] = np.degrees(th_ref).round(3).tolist()

    worst = run(m, d, lambda t: th_ref, 0.4)
    p = d.xpos[m.body("platform").id]
    out["2_hold_drift_mm"] = float(np.linalg.norm(p - p_ref) * 1000)
    out["2_hold_max_closure_residual_mm"] = worst * 1000

    errs = []
    for target in ([0.05, 0.03, -0.26], [-0.08, -0.0865, -0.24], [0.075, 0.0865, -0.215], [0.0, 0.0, -0.30]):
        th_goal, ok = DELTA.ik(np.array(target))
        th0 = np.array([d.qpos[m.joint(f"j{n}").qposadr[0]] for n in (1, 2, 3)])
        T = 0.35

        def traj(t, t0=d.time, th0=th0, th1=th_goal):
            s = np.clip((t - t0) / T, 0, 1)
            s = s * s * s * (10 - 15 * s + 6 * s * s)
            return th0 + (th1 - th0) * s
        worst = max(worst, run(m, d, traj, T + 0.3))
        p = d.xpos[m.body("platform").id]
        quat = d.xquat[m.body("platform").id]
        tilt = 2 * np.degrees(np.arccos(min(1.0, abs(quat[0]))))
        th_now = np.array([d.qpos[m.joint(f"j{n}").qposadr[0]] for n in (1, 2, 3)])
        x_fk = DELTA.fk(th_now)
        errs.append(dict(target_mm=(np.array(target) * 1000).round(1).tolist(), reached_mm=(p * 1000).round(3).tolist(),
                         error_vs_target_mm=round(float(np.linalg.norm(p - np.array(target)) * 1000), 3),
                         error_vs_fk_of_measured_angles_mm=round(float(np.linalg.norm(p - np.asarray(x_fk)) * 1000), 4),
                         platform_tilt_deg=round(float(tilt), 4)))
    out["3_moves"] = errs
    out["3_max_closure_residual_mm"] = worst * 1000
    out["model"] = dict(nbody=m.nbody, njnt=m.njnt, nq=m.nq, nv=m.nv, neq=m.neq, ntendon=m.ntendon, nu=m.nu,
                        masses_g={m.body(i).name: round(float(m.body_mass[i]) * 1000, 2) for i in range(1, m.nbody)},
                        armature_hinge=float(m.dof_armature[m.joint("j1").dofadr[0]]))
    print(json.dumps(out, indent=1))
    json.dump(out, open(os.path.join(HERE, "out", "mujoco", "validation.json"), "w"), indent=1)
    ok = (out["1_closure_residual_at_reference_mm"] < 0.01 and out["2_hold_drift_mm"] < 0.5
          and all(e["error_vs_fk_of_measured_angles_mm"] < 0.05 for e in errs))
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
