# PhysicsManager.py
"""
목표 : 전체 물리 엔진 및 모듈 통합 관리
최초 작성일자 : 2025-05-14

기여자 : 박도솔

최종 수정자 : 박도솔
최종 수정일자 : 2025-05-25

참고 : VehicleDynamics, Brake, Pacejka 통합 
"""

from .VehicleDynamics import VehicleDynamics
from .Brake import AirBrake
from .Pacejka import Pacejka
from input.controller import KeyboardController
from utils.load_data import *

class PhysicsManager:
    """
    전체 물리 엔진 및 각 모듈(차량 동역학, 타이어, 서스펜션 등) 통합 관리 클래스
    """

    # TODO : 서스펜션 정적 처짐 적용 - 호출 시켜서 사용되게 해야함 !!! 
    @staticmethod
    def apply_static_sag(model, data, UNSPRUNG_MASS=150, SPRING_K=500000): 
        STATIC_SAG = UNSPRUNG_MASS * 9.81 / SPRING_K
        # 서스펜션 정적 처짐 적용
        for j_name in ("fl_susp", "fr_susp", "rl_susp", "rr_susp"):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j_name)
            data.qpos[jid] = -STATIC_SAG
        mujoco.mj_forward(model, data)  # 초기 상태 강제 계산

    def __init__(self, model, data, params, ctrl):
        self.model = model
        self.data = data
        self.params = params
        self.ctrl = ctrl

        self.vehicle = VehicleDynamics(self.model, self.params)
        self.brake = AirBrake(self.model, self.params)
        self.pacejka = Pacejka(self.model, self.params)

        self.time = 0.0

        self.apply_static_sag(self.model, self.data)

    def step(self):
        self.ctrl.apply(self.data)
        # self.pacejka.apply(self.model, self.params) # TODO
        self.brake.apply(self.data, self.params)
        self.vehicle.apply(self.data, self.params)

    def get_state(self):
        return {
            "time": self.time,
            "position": self.data.qpos,
            "velocity": self.data.qvel,
            "acceleration": self.data.qacc,
            "angular_velocity": self.data.qvel[3:],
            "angular_acceleration": self.data.qacc[3:],
            "brake_torque": self.brake.brake_torque,
            "brake_pressure": self.brake.brake_pressure,
            "brake_slip": self.brake.brake_slip,
            "wheel_speed": self.data.qvel[self.wheel_ids],
            "wheel_slip": self.data.qvel[self.wheel_ids],
            "wheel_load": self.data.qvel[self.wheel_ids],
            "wheel_force": self.data.qvel[self.wheel_ids],
            "wheel_torque": self.data.qvel[self.wheel_ids],
            "wheel_acceleration": self.data.qvel[self.wheel_ids],
        }