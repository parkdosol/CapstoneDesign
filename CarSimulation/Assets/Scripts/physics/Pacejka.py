# Pacejka.py
"""
목적 : 파제카 마찰 함수
기여자 : 김준호, 박도솔, 박진석, 윤건영
최초 작성일자 : 2025-05-03
최종 수정자 : 윤건영
최종 수정일자 : 2025-06-01
"""

import numpy as np
import mujoco

from utils.load_data import *

# ────────────────────────────────────────────────────────────────────────────
class Pacejka:

    @staticmethod
    def _magic_formula(slip: np.ndarray | float, Fz: float,
                    B: float, C: float, E: float, mu: float,
                    Dmax: float | None = None) -> np.ndarray | float:
        """Pacejka Magic Formula (Fx 또는 Fy 계산)"""
        D = mu * Fz
        if Dmax is not None:
            D = np.minimum(D, Dmax)
        Bs = B * slip
        return D * np.sin(C * np.arctan(Bs - E * (Bs - np.arctan(Bs))))
    
    def __init__(self, model: mujoco.MjModel, params: dict):
        self.model = model
        self.params = params
        
        self.cid = get_chassis_id(model)
        self.wids = get_wheel_ids(model)
        self.mass = model.body_mass[self.cid]
        self.h_cg = model.body_pos[self.cid, 2] if self.params["CG_HEIGHT"] is None else self.params["CG_HEIGHT"]

    

    def apply(self, data: mujoco.MjData, brake=None):
        # (1) 직전 스텝 힘/토크 초기화
        for _, bid, jid, *_ in self.ids:
            data.xfrc_applied[bid] = 0
            data.qfrc_applied[jid] = 0

        # (2) 차체 가속도(local) 계산 → 하중 이동
        acc_world = np.array([data.qacc[0], data.qacc[1], data.qacc[2]])
        cid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        R_cg = data.xmat[cid].reshape(3,3)
        ax, ay = (R_cg.T @ acc_world)[:2]
        Fz_static = self.mass * 9.81 / 4

        for label, bid, jid, sjid, aid in self.ids:
            front = label.startswith("F")
            left  = label.endswith("L")
            dF_long = self.mass * ax * self.h_cg / WHEEL_BASE
            dF_lat  = self.mass * ay * self.h_cg / (2*TRACK_HALF)
            Fz = float(np.clip(Fz_static + 0.5*( dF_long*(1 if front else -1)
                                               +dF_lat *(1 if left  else -1)),
                                10.0, MAX_LOAD))

            # 바디 로컬 속도 (vx, vy)
            vel6 = np.zeros(6,float)
            mujoco.mj_objectVelocity(self.model, data, mujoco.mjtObj.mjOBJ_BODY, bid, vel6, 1)
            vx, vy = float(vel6[3]), float(vel6[4])

            # 슬립비 κ 계산
            omega = float(data.qvel[jid])
            vw  = omega * WHEEL_RADIUS
            vn  = max(abs(vx), 0.5)
            τcmd = float(data.ctrl[aid])
            if abs(omega) < 1e-3 and abs(τcmd) > 10:
                kappa = np.sign(τcmd) * 0.1
            else:
                kappa = np.clip((vw - vx) / vn, -SLIP_LIMIT, SLIP_LIMIT)

            # 슬립각 β 계산
            yaw_rate = float(data.qvel[5])
            dy       = WHEEL_BASE/2 if front else -WHEEL_BASE/2
            steer    = float(data.qpos[sjid]) if sjid is not None else 0.0
            if abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(yaw_rate) < 1e-3:
                beta = 0.0
            else:
                beta = np.clip(np.arctan2(vy + dy*yaw_rate, vn) - steer,
                               -SLIP_ANGLE_LIMIT, SLIP_ANGLE_LIMIT)

            # 접지력 계산 (휠 로컬)
            Fx = float(np.clip(_magic_formula(kappa, Fz, BX, CX, EX, MU_X), -MU_X*MAX_LOAD, MU_X*MAX_LOAD))
            Fy = float(np.clip(_magic_formula(beta,  Fz, BY, CY, EY, MU_Y), -MU_Y*MAX_LOAD, MU_Y*MAX_LOAD))

            # ── 로컬 → 월드 변환 후 적용 ─────────────────
            Rw = data.xmat[bid].reshape(3,3)
            data.xfrc_applied[bid,:3] = Rw @ np.array([Fx, Fy, 0])

            # 휠 축 토크 (모터 반력+저항) 적용
            tq = np.sign(omega)*Fx*WHEEL_RADIUS + JOINT_DAMPING*omega
            data.qfrc_applied[jid] -= tq

        # (3) 브레이크 토크 직접 반영
        if brake and brake.braking:
            for (label, bid, jid, sjid, aid), act_idx in zip(self.ids, brake._act_brake):
                data.qfrc_applied[jid] += float(data.ctrl[act_idx])
