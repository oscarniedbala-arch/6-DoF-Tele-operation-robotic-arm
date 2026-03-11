"Analytical and DLS IK for tw2ka's robotic arm"
"the arm is inspired by a KUKA KR Cybertech 6-DOF arm, but the dimensions and joint limits are not exact and are just for testing purposes. "
"The arm has a 3-DOF shoulder-elbow-wrist structure, with a roll-pitch-roll wrist configuration. "
"The IK solutions are computed using both an analytical approach (for the 3-DOF shoulder-elbow part) and a numerical DLS method (for the full 6-DOF problem)."


"This is only a kinematics module and does not include any dynamics, control, or integration with ROS or Gazebo. "
"Hopfully i can add ROS2 intergration soon but my main focus will be the TeleOp side of things, so this module is just for testing and demonstration purposes. "

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import math
import numpy as np


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def _norm(v: np.ndarray) -> float:
    return float(np.linalg.norm(v))


def rot_x(a: float) -> np.ndarray:
    ca, sa = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, ca, -sa],
                     [0.0, sa,  ca]], dtype=float)

def rot_y(a: float) -> np.ndarray:
    ca, sa = math.cos(a), math.sin(a)
    return np.array([[ ca, 0.0, sa],
                     [0.0, 1.0, 0.0],
                     [-sa, 0.0, ca]], dtype=float)

def rot_z(a: float) -> np.ndarray:
    ca, sa = math.cos(a), math.sin(a)
    return np.array([[ca, -sa, 0.0],
                     [sa,  ca, 0.0],
                     [0.0, 0.0, 1.0]], dtype=float)


def T_from_Rp(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = p.reshape(3)
    return T

def T_inv(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    p = T[:3, 3]
    Ti = np.eye(4, dtype=float)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ p
    return Ti


def so3_log(R: np.ndarray) -> np.ndarray:
    tr = float(np.trace(R))
    c = _clamp((tr - 1.0) * 0.5, -1.0, 1.0)
    th = math.acos(c)
    if th < 1e-12:
        return np.zeros(3, dtype=float)
    w_hat = (R - R.T) / (2.0 * math.sin(th))
    return np.array([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]], dtype=float) * th


def pose_error(T_cur: np.ndarray, T_tgt: np.ndarray) -> np.ndarray:
    Te = T_inv(T_cur) @ T_tgt
    w = so3_log(Te[:3, :3])
    p = Te[:3, 3]
    return np.hstack([w, p])


@dataclass(frozen=True)
class JointLimit:
    lo: float
    hi: float

    def clamp(self, q: float) -> float:
        return _clamp(float(q), float(self.lo), float(self.hi))


@dataclass(frozen=True)
class KRCybertechLike:
    d1: float = 0.05   # m (base->shoulder axis height)
    L2: float = 0.16   # m (shoulder->elbow)
    L3: float = 0.14   # m (elbow->wrist center)
    d6: float = 0.02   # m (flange->TCP along tool Z)

    qlim: Tuple[JointLimit, ...] = (
        JointLimit(math.radians(-90),  math.radians(90)),     # J1
        JointLimit(math.radians(-15),  math.radians(110)),    # J2
        JointLimit(math.radians(-45),  math.radians(45)),     # J3
        JointLimit(math.radians(-180), math.radians(180)),    # J4
        JointLimit(math.radians(-90),  math.radians(90)),     # J5
        JointLimit(math.radians(-90),  math.radians(90)),     # J6
    )


def clamp_q(geom: KRCybertechLike, q: np.ndarray) -> np.ndarray:
    out = q.copy()
    for i, lim in enumerate(geom.qlim):
        out[i] = lim.clamp(out[i])
    return out


def fk(geom: KRCybertechLike, q: Sequence[float]) -> np.ndarray:
    q1, q2, q3, q4, q5, q6 = [float(x) for x in q]

    T0_1 = T_from_Rp(rot_z(q1), np.array([0.0, 0.0, 0.0]))
    T1_2 = T_from_Rp(np.eye(3), np.array([0.0, 0.0, geom.d1]))

    T2_3 = T_from_Rp(rot_y(q2), np.zeros(3))
    T3_4 = T_from_Rp(np.eye(3), np.array([geom.L2, 0.0, 0.0]))

    T4_5 = T_from_Rp(rot_y(q3), np.zeros(3))
    T5_wc = T_from_Rp(np.eye(3), np.array([geom.L3, 0.0, 0.0]))

    # wrist: roll–pitch–roll (X–Y–X) at the wrist center
    Rw = rot_x(q4) @ rot_y(q5) @ rot_x(q6)
    T_wc_6 = T_from_Rp(Rw, np.zeros(3))

    T6_tcp = T_from_Rp(np.eye(3), np.array([0.0, 0.0, geom.d6]))

    return T0_1 @ T1_2 @ T2_3 @ T3_4 @ T4_5 @ T5_wc @ T_wc_6 @ T6_tcp


def jacobian_numeric(geom: KRCybertechLike, q: Sequence[float], eps: float = 1e-6) -> np.ndarray:
    q = np.array(q, dtype=float).reshape(6)
    T0 = fk(geom, q)
    J = np.zeros((6, 6), dtype=float)
    for i in range(6):
        dq = q.copy()
        dq[i] += eps
        Ti = fk(geom, dq)
        J[:, i] = pose_error(T0, Ti) / eps
    return J


def _wrist_decompose_x_y_x(R: np.ndarray) -> List[Tuple[float, float, float]]:
    # R = Rx(a) Ry(b) Rx(c)
    # b in [0, pi] via acos(R00); second solution uses -b.
    c0 = _clamp(float(R[0, 0]), -1.0, 1.0)
    b1 = math.acos(c0)
    b2 = -b1

    sols: List[Tuple[float, float, float]] = []

    for b in (b1, b2):
        sb = math.sin(b)
        if abs(sb) < 1e-9:
            # singular: b ~ 0 (or pi). Then R ≈ Rx(a+c) (or Rx(a-c) depending on pi branch).
            # Pick a=0, c from remaining submatrix.
            a = 0.0
            c = math.atan2(float(R[2, 1]), float(R[2, 2]))
            sols.append((a, b, c))
            continue

        c = math.atan2(float(R[0, 1]), float(R[0, 2]))
        a = math.atan2(float(R[1, 0]), float(-R[2, 0]))
        sols.append((a, b, c))

    return sols


def ik_analytic(
    geom: KRCybertechLike,
    T_target: np.ndarray,
    q_seed: Optional[Sequence[float]] = None,
) -> List[np.ndarray]:
    if q_seed is None:
        q_seed = [0.0] * 6
    q_seed = np.array(q_seed, dtype=float).reshape(6)

    R = T_target[:3, :3]
    p = T_target[:3, 3]

    p_wc = p - geom.d6 * R[:, 2]

    x, y, z = float(p_wc[0]), float(p_wc[1]), float(p_wc[2])

    q1 = math.atan2(y, x)

    r = math.hypot(x, y)
    z2 = z - geom.d1

    L2, L3 = geom.L2, geom.L3
    D = (r*r + z2*z2 - L2*L2 - L3*L3) / (2.0 * L2 * L3)
    if D < -1.0 - 1e-8 or D > 1.0 + 1e-8:
        return []

    D = _clamp(D, -1.0, 1.0)
    q3_candidates = [math.acos(D), -math.acos(D)]

    sols: List[np.ndarray] = []

    for q3 in q3_candidates:
        k1 = L2 + L3 * math.cos(q3)
        k2 = L3 * math.sin(q3)
        q2 = math.atan2(z2, r) - math.atan2(k2, k1)

        # R0_3 from our arm convention
        R0_3 = rot_z(q1) @ rot_y(q2) @ rot_y(q3)

        R3_6 = R0_3.T @ R

        for q4, q5, q6 in _wrist_decompose_x_y_x(R3_6):
            q = np.array([q1, q2, q3, q4, q5, q6], dtype=float)
            q = np.array([_wrap_pi(float(a)) for a in q], dtype=float)

            ok = True
            for i, lim in enumerate(geom.qlim):
                if q[i] < lim.lo - 1e-6 or q[i] > lim.hi + 1e-6:
                    ok = False
                    break
            if ok:
                sols.append(q)

    if not sols:
        return []

    def score(q: np.ndarray) -> float:
        dq = np.array([_wrap_pi(q[i] - q_seed[i]) for i in range(6)], dtype=float)
        return float(dq @ dq)

    sols = sorted(sols, key=score)
    return sols


@dataclass
class IKDLSSettings:
    max_iters: int = 250
    tol_pos: float = 5e-4
    tol_rot: float = 5e-4
    damping: float = 1e-2
    step: float = 1.0


def ik_dls(
    geom: KRCybertechLike,
    T_target: np.ndarray,
    q0: Sequence[float],
    cfg: IKDLSSettings = IKDLSSettings(),
) -> Tuple[np.ndarray, bool]:
    q = np.array(q0, dtype=float).reshape(6)
    for _ in range(int(cfg.max_iters)):
        Tcur = fk(geom, q)
        e = pose_error(Tcur, T_target)
        if _norm(e[:3]) <= cfg.tol_rot and _norm(e[3:]) <= cfg.tol_pos:
            return clamp_q(geom, q), True

        J = jacobian_numeric(geom, q)
        lam = float(cfg.damping)
        A = J @ J.T + (lam * lam) * np.eye(6)
        dq = J.T @ np.linalg.solve(A, e)

        q = q + float(cfg.step) * dq
        q = np.array([_wrap_pi(float(a)) for a in q], dtype=float)
        q = clamp_q(geom, q)

    return clamp_q(geom, q), False


def make_target_pose(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    R = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
    return T_from_Rp(R, np.array([x, y, z], dtype=float))


if __name__ == "__main__":
    geom = KRCybertechLike()
    Tt = make_target_pose(0.22, 0.00, 0.12, 0.0, math.radians(30), 0.0)

    sols = ik_analytic(geom, Tt, q_seed=[0, 0, 0, 0, 0, 0])
    if sols:
        q = sols[0]
        print("analytic q (deg):", [round(math.degrees(a), 2) for a in q])
        print("fk p:", fk(geom, q)[:3, 3])
    else:
        q, ok = ik_dls(geom, Tt, q0=[0, 0, 0, 0, 0, 0])
        print("dls ok:", ok, "q (deg):", [round(math.degrees(a), 2) for a in q])
