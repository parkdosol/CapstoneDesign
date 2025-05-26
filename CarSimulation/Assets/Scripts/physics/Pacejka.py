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
        pass
