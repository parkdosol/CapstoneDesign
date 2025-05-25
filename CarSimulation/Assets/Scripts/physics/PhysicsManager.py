# PhysicsManager.py
"""
목표 : 전체 물리 엔진 및 모듈 통합 관리
최초 작성자 : 박도솔
최초 작성일자 : 2025-05-14

기여자 : 박도솔

최종 수정자 : 박도솔
최종 수정일자 : 2025-05-25

참고 : VehicleDynamics, Brake, Pacejka 통합 
"""

from .VehicleDynamics import VehicleDynamics
from .Brake import AirBrake
from .Pacejka import Pacejka
from utils.load_data import *

class PhysicsManager:
    """
    전체 물리 엔진 및 각 모듈(차량 동역학, 타이어, 서스펜션 등) 통합 관리 클래스
    """

    # 서스펜션 정적 처짐 적용 
    @staticmethod
    def apply_static_sag(model, data): 
        # 서스펜션 정적 처짐 적용
        for j_name in ("fl_susp", "fr_susp", "rl_susp", "rr_susp"):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j_name)
            data.qpos[jid] = -STATIC_SAG
        mujoco.mj_forward(model, data)  # 초기 상태 강제 계산

    def __init__(self):
        self.params = load_json()
        self.model, self.data = load_model(self.params)

        self.vehicle = VehicleDynamics(self.model)
        self.brake = AirBrake(self.model)
        self.pacejka = Pacejka(self.model)

        self.time = 0.0

        self.apply_static_sag(self.model, self.data)



    # TODO : 서스펜션 정적 처짐 적용 (팀원들은 main.py에 선언)

    def step(self):
        # TODO : 서스펜션 정적 처짐 적용 

        # TODO : pacejka 적용 

        # TODO : brake 적용 

        # TODO : vehicle dynamics 적용