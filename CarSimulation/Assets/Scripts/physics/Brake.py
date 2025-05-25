# Brake.py
"""
목적 : 에어 브레이크 + ABS 시스템
기여자 : 김준호, 박도솔, 박진석, 윤건영 
최종 수정자 : 박도솔 
최종 수정일자 : 2025-05-25

참고 : main loop 실행시 여러번 호출하지 않도록 최적화 필요함 
"""

import numpy as np
import mujoco
import glfw

from utils.load_data import *

PARAMS = load_json()

# ────────────────────────────────────────────────────────────────────────────
class AirBrake:
    def __init__(self,model: mujoco.MjModel) -> None:
        
        # 필요한 ID 캐싱 및 내부 상태 초기화
        # TODO : main loop 실행시 id 호출 여러번 하지 않도록 최적화 필요

        self.model = model

        self.cid = get_chassis_id(model)        # 차체 ID
        self.wids = get_wheel_ids(model)       # 바퀴 ID
        self.mass = model.body_mass[self.cid] # 차체 질량
        self.h_cg = model.body_pos[self.cid, 2] if PARAMS["CG_HEIGHT"] is None else PARAMS["CG_HEIGHT"] # 차체 중심 높이

        # 내부 상태 변수
        self.p = PARAMS["BRAKE_PARAMS"]["INITIAL_PRESSURE"]
        self.braking = PARAMS["FLAG_PARAMS"]["BREAK_MODE"]
        self.current_speed = 0.0 #최근 계산 속도 [m/s]
        self.brake_torque = 0.0 #최근 계산 토크 [N·m]
        self.last_torque = [0.0, 0.0, 0.0, 0.0] #각 바퀴별 최근 브레이크 토크 저장
        self.last_pressure = [0.0, 0.0, 0.0, 0.0] #각 바퀴별 최근 압력 [Pa]

    def apply_brake(self, data: mujoco.MjData, drive_rpm: float = 1000.0) -> None:
        
        # TODO : main loop 실행시 여러번 호출하지 않도록 최적화 필요 

        


