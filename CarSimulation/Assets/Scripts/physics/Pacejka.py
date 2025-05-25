# Pacejka.py
"""
목적 : 파제카 마찰 함수
기여자 : 김준호, 박도솔, 박진석, 윤건영
최초 작성일자 : 2025-05-03
최종 수정자 : 박도솔
최종 수정일자 : 2025-05-25
"""

import numpy as np
import mujoco

from utils.load_data import *

PARAMS = load_json()

# ────────────────────────────────────────────────────────────────────────────
class Pacejka:
    @staticmethod
    def _magic_formula(slip: np.ndarray | float, Fz: float,
                       B: float, C: float, E: float, mu: float,
                       Dmax: float | None = None) -> np.ndarray | float:
        return mu * Fz * (A * slip + B * np.sin(C * np.arctan(B * slip - E * (B * slip - np.arctan(B * slip)))))

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.cid = get_chassis_id(model)
        self.wids = get_wheel_ids(model)
        self.mass = model.body_mass[self.cid]
        self.h_cg = model.body_pos[self.cid, 2] if PARAMS["CG_HEIGHT"] is None else PARAMS["CG_HEIGHT"]
    

    def apply(self, data: mujoco.MjData, brake=None):
        # (1) 브레이크 토크 백업 & 직전 스텝 힘/토크 초기화
        bk_torque: dict[int, float] = {}

        for _, bid, jid, _, _ in self.wids:
            # 초기화
            data.xfrc_applied[bid] = 0 # 차체 힘
            data.qfrc_applied[jid] = 0 # 바퀴 힘

            # 브레이크 토크 백업 (필요 시만)
            if brake and brake.braking:
                bk_torque[jid] = float(data.qfrc_applied[jid])

        # (2) 차체 가속도(local) 계산 → 하중 이동

        # TODO : Main loop에서 최적화 될 수 있도록 코드 수정 필요