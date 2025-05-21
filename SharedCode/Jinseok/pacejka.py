# pacejka_model.py
"""
목적 : 파제카 마찰 함수
작성자 : 박진석
최초 작성일자 : 2025-05-03
수정자 : 박진석
최종 수정일자 : 2025-05-21
"""

import numpy as np
import mujoco

# ───────────────── 파라미터 블록 ─────────────────
# (필요에 따라 이 값들만 조정하면 됩니다.)

# ► 차량 제원
WHEEL_BASE      = 2.50   # m   휠베이스
TRACK_HALF      = 3.00   # m   트랙 반차폭
CG_HEIGHT       = 0.90   # m   무게중심 높이 (model에서 읽으면 치환됨)
WHEEL_RADIUS    = 0.35   # m
JOINT_DAMPING   = 0.10   # N·m·s/rad  (휠 축 마찰)

# ► Pacejka 종/횡 계수 (Bx,Cx,Ex,μ)
BX, CX, EX, MU_X = 12.0, 1.9, 0.97, 0.9   # Longitudinal
BY, CY, EY, MU_Y =  8.5, 1.3, 0.97, 1.0   # Lateral
MAX_LOAD         = 3000.0  # N  휠당 최대 하중(클램프)

# ► 슬립 한계
SLIP_LIMIT       = 0.20   # |κ| ≤ 0.2 (±20%)
SLIP_ANGLE_LIMIT = 0.26   # |β| ≤ 15°  (0.26 rad)

# ───────────────── Helper 함수 ───────────────────

def _magic_formula(slip: np.ndarray | float, Fz: float,
                   B: float, C: float, E: float, mu: float,
                   Dmax: float | None = None) -> np.ndarray | float:
    """Pacejka Magic Formula (Fx 또는 Fy 계산)"""
    D = mu * Fz
    if Dmax is not None:
        D = np.minimum(D, Dmax)
    Bs = B * slip
    return D * np.sin(C * np.arctan(Bs - E * (Bs - np.arctan(Bs))))

# ────────────────────────────────────────────────
class Pacejka:
    """네 바퀴 Pacejka 접지력 계산기"""

    def __init__(self, model: mujoco.MjModel):
        self.model = model

        # ── 바퀴·조향·모터 ID 캐싱 ──
        wheel_defs = [
            ("FL", "front_left_wheel",  "fl_wheel", "fl_steer"),
            ("FR", "front_right_wheel", "fr_wheel", "fr_steer"),
            ("RL", "rear_left_wheel",   "rl_wheel", None),
            ("RR", "rear_right_wheel",  "rr_wheel", None),
        ]
        self.ids: list[tuple[str,int,int,int|None,int]] = []
        for label, body, wj, sj in wheel_defs:
            bid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,  body)
            jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, wj)
            sjid = (mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, sj)
                     if sj else None)
            aid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, wj.replace("_wheel","_motor"))
            self.ids.append((label, bid, jid, sjid, aid))

        # ── 차체 기본 제원 ──
        cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self.mass     = float(model.body_mass[cid])
        self.h_cg     = float(model.body_pos[cid,2]) if CG_HEIGHT is None else CG_HEIGHT

    # ---------------------------------------------------------
    def apply(self, data: mujoco.MjData, brake=None):
        """매 스텝 호출하여 접지력·모터토크 계산"""
        # (1) 브레이크 토크 백업
        bk_torque: dict[int,float] = {}
        if brake and brake.braking:
            for _,_,jid,_,_ in self.ids:
                bk_torque[jid] = float(data.qfrc_applied[jid])

        # (2) 직전 스텝 힘/토크 초기화
        for _, bid, jid, *_ in self.ids:
            data.xfrc_applied[bid] = 0
            data.qfrc_applied[jid] = 0

        # (3) 차체 가속도(local) 계산 → 하중 이동
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
            # 속도와 요 레이트가 매우 작으면 beta를 0으로 강제 설정
            if abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(yaw_rate) < 1e-3:
                beta = 0.0
            else:
                beta = np.clip(np.arctan2(vy + dy*yaw_rate, vn) - steer,
                               -SLIP_ANGLE_LIMIT, SLIP_ANGLE_LIMIT)

            # 접지력 계산 (휠 로컬 좌표계)
            Fx = float(np.clip(_magic_formula(kappa, Fz, BX, CX, EX, MU_X), -MU_X*MAX_LOAD, MU_X*MAX_LOAD))
            Fy = float(np.clip(_magic_formula(beta,  Fz, BY, CY, EY, MU_Y), -MU_Y*MAX_LOAD, MU_Y*MAX_LOAD))

            # ── 로컬 → 월드 변환 후 적용 ─────────────────
            Rw = data.xmat[bid].reshape(3,3)
            data.xfrc_applied[bid,:3] = Rw @ np.array([Fx, Fy, 0])
            # ------------------------------------------------

            # 휠 축 토크 (모터 반력+저항) 적용
            tq = np.sign(omega)*Fx*WHEEL_RADIUS + JOINT_DAMPING*omega
            data.qfrc_applied[jid] -= tq

        # (4) 브레이크 토크 복원
        if brake and brake.braking:
            for jid, tq in bk_torque.items():
                data.qfrc_applied[jid] += tq
