"""
목적   : 차량 HUD(Head-Up Display) 정보 실시간 출력 (차체 좌표계 처리)
작성자 : 
최초 작성일자 : 2025-05-03
수정자 : 박진석
최종 수정일자 : 2025-05-21
"""

# ───────────────── 파라미터 블록 ─────────────────
# ***** 모델 의존 식별자 *****
WHEEL_BODY_NAMES: dict[str, str] = {
    "FL": "front_left_wheel",
    "FR": "front_right_wheel",
    "RL": "rear_left_wheel",
    "RR": "rear_right_wheel",
}
WHEEL_JOINT_NAMES: tuple[str, ...] = (
    "fl_wheel",
    "fr_wheel",
    "rl_wheel",
    "rr_wheel",
)
CHASSIS_BODY_NAME = "chassis"  # 차체 바디 이름

# ***** 단위 변환 계수 *****
N_TO_kN  = 1.0 / 1000.0  # N → kN
M_TO_KMH = 3.6           # m/s → km/h

# ***** HUD 표시 옵션 *****
SHOW_MASS_INFO = True

# ──────────────────────────────────────────────────

import time
from typing import List, Tuple
import numpy as np
import mujoco


class VehicleMonitor:
    """차량 상태를 수집해 HUD 문자열로 반환"""

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.t0_wall = time.perf_counter()

        # 바디·조인트 ID 캐싱
        self.wheels: List[Tuple[str, int]] = [
            (lbl, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bname))
            for lbl, bname in WHEEL_BODY_NAMES.items()
        ]
        self.wheel_jid: List[int] = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            for jname in WHEEL_JOINT_NAMES
        ]
        self.chassis_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, CHASSIS_BODY_NAME)
        if self.chassis_bid == -1:
            self.chassis_bid = 0  # fallback: body 0 (world) → 좌표계 변환 없음

        self.mass_t = float(sum(model.body_mass)) / 1000.0
        self.reset()

    # ----------------------------------------------
    def reset(self):
        self.thrtl = self.steer = 0.0
        self.brake_p = 0.0
        self.speed = 0.0
        self.fx = [0.0]*4; self.fy = [0.0]*4; self.fz = [0.0]*4
        self.omega = [0.0]*4
        self.t_wall = self.t_sim = 0.0

    # ----------------------------------------------
    @staticmethod
    def _fmt(val: float | int, width: int = 7, dec: int = 1) -> str:
        s = f"{abs(val):0{width-1}.{dec}f}" if isinstance(val, float) else f"{abs(val):0{width-1}d}"
        return ("-" if val < 0 else " ") + s

    # ----------------------------------------------
    def update(self, model: mujoco.MjModel, data: mujoco.MjData, brake, ctrl) -> None:
        # ── 입력 값 수집 ──
        self.thrtl   = ctrl.torque
        self.steer   = ctrl.steering
        self.brake_p = brake.p * 0.001  # Pa → kPa
        self.speed   = float(np.linalg.norm(data.qvel[0:2]) * M_TO_KMH)
        self.t_sim   = data.time
        self.t_wall  = time.perf_counter() - self.t0_wall

        # 월드→차체 회전행렬 (3×3)
        R_c2w = data.xmat[self.chassis_bid].reshape(3,3)
        R_w2c = R_c2w.T  # 전치 = 역행렬 (정규 직교)

        # 바퀴별 힘·각속도 (차체 기준)
        for i, (lbl, bid) in enumerate(self.wheels):
            # Fx, Fy는 xfrc_applied에서 가져옴 (Pacejka 힘)
            f_world = data.xfrc_applied[bid, 0:3]
            fx_c, fy_c, _ = R_w2c @ f_world * N_TO_kN
            # Fy가 미세하면 0으로 보정
            fy_c = 0.0 if abs(fy_c) < 1e-3 else float(fy_c)

            # Fz는 센서에서 접촉력으로 가져옴
            sensor_name = f"{lbl.lower()}_force"
            sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
            if sensor_id != -1:
                sensor_adr = model.sensor_adr[sensor_id]
                sensor_dim = model.sensor_dim[sensor_id]
                f_contact = np.array(data.sensordata[sensor_adr:sensor_adr + sensor_dim])
                _, _, fz_c = R_w2c @ f_contact * N_TO_kN
                # Fz 방향 보정 (음수면 양수로 변환)
                fz_c = abs(fz_c)
            else:
                fz_c = 0.0

            self.fx[i], self.fy[i], self.fz[i] = float(fx_c), float(fy_c), float(fz_c)

            jid = self.wheel_jid[i]
            if jid == -1:
                self.omega[i] = float('nan')
            else:
                self.omega[i] = float(data.qvel[jid])

    # ----------------------------------------------
    def text(self) -> str:
        p = self._fmt
        lines: list[str] = [
            "INPUT  Thrtl:" + p(self.thrtl,6,0) +
            "  Steer:" + p(self.steer,6,0) +
            f"  Brake:{int(self.brake_p):04d}kPa",

            f"VEHCL  Speed:{self.speed:6.1f}km/h" + (f"  Mass:{self.mass_t:4.1f}t" if SHOW_MASS_INFO else ""),

            "FORCE  Fx:" + p(sum(self.fx)) + "kN  Fy:" + p(sum(self.fy)) + "kN",

            f"TIME   Sim:{self.t_sim:6.1f}s  Real:{self.t_wall:6.1f}s",
            "",
            "WHEEL  w(rad/s) & Force[kN]",
            "      w     Fx     Fy     Fz",
        ]
        for i, (lbl, _) in enumerate(self.wheels):
            fx, fy, fz = self.fx[i], self.fy[i], self.fz[i]
            omega = self.omega[i]
            w_str = "  nan" if np.isnan(omega) else p(omega)
            lines.append(f" {lbl} " + w_str + " " + p(fx) + " " + p(fy) + " " + p(fz))
        return "\n".join(lines)