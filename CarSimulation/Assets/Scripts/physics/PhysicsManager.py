"""
목표 : 전체 물리 엔진 및 모듈 통합 관리
최초 작성자 : 박도솔
최초 작성일자 : 2025-05-14

기여자 : 박도솔

최종 수정자 : 박도솔
최종 수정일자 : 2025-05-14

참고 : VehicleDynamics, Tire, Suspension 등 통합
"""

from .VehicleDynamics import VehicleDynamics
from utils.load_data import load_params_from_json

class PhysicsManager:
    """
    전체 물리 엔진 및 각 모듈(차량 동역학, 타이어, 서스펜션 등) 통합 관리 클래스
    """
    
    def __init__(self, car_params, tire_params=None, suspension_params=None):
        self.vehicle = VehicleDynamics(car_params, tire_params, suspension_params)
        self.time = 0.0

    def step(self, control_input, dt):
        """
        시뮬레이션 한 스텝 진행 (외부 입력 및 시간 간격 반영)
        """
        self.vehicle.update(control_input, dt)
        self.time += dt

    def get_vehicle_state(self):
        """
        현재 차량 상태 반환 (위치, 속도, 가속도 등)
        """
        return {
            'position': self.vehicle.position.copy(),
            'velocity': self.vehicle.velocity.copy(),
            'acceleration': self.vehicle.acceleration.copy(),
            'yaw': self.vehicle.yaw
        } 