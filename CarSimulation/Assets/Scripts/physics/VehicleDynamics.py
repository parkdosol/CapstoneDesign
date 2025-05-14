"""
목표 : 차량 물리 코드 통합 관리 코드 
최초 작성자 : 박도솔
최초 작성일자 : 2025-05-14

기여자 : 박도솔

최종 수정자 : 박도솔
최종 수정일자 : 2025-05-14

참고 : 수정 작업중
"""
import numpy as np
from .Tire import PacejkaTireModel
from .Suspension import Suspension
from utils.load_data import load_params_from_json

class VehicleDynamics:
    """
    차량 전체 동역학을 관리하는 클래스
    """
    def __init__(self, car_params, tire_params=None, suspension_params=None):
        # 차량 파라미터 초기화
        self.car_params = car_params
        self.mass = car_params['mass']
        self.wheelbase = car_params['wheelbase']
        self.track = car_params['track']

        # 타이어 및 서스펜션 모델 초기화
        self.tire_model = PacejkaTireModel(car_params, tire_params)
        self.suspension = Suspension(**(suspension_params or {}))
        
        # 차량 상태 변수 예시 (실제 시뮬레이션 연동 시 확장 필요)
        self.velocity = np.zeros(3)  # [vx, vy, vz]
        self.acceleration = np.zeros(3)
        self.position = np.zeros(3)
        self.yaw = 0.0

    def update(self, control_input, dt):
        """
        control_input: {'throttle': float, 'brake': float, 'steer': float}
        dt: 시간 간격 (초)
        """
        # 예시: 단순 전진/조향/브레이크 동역학 (실제 구현 시 확장 필요)
        throttle = control_input.get('throttle', 0.0)
        brake = control_input.get('brake', 0.0)
        steer = control_input.get('steer', 0.0)
        
        # 타이어 힘 계산 (간단화)
        Fz = self.mass * 9.81 / 4.0  # 각 바퀴 하중 (정적 분포)
        slip_ratio = throttle - brake  # 실제로는 속도/구동력 기반 계산 필요
        slip_angle = steer  # 실제로는 속도/조향각 기반 계산 필요
        Fx, Fy = self.tire_model.compute_forces(slip_ratio, slip_angle, Fz)

        # 가속도 계산 (단순화)
        ax = Fx / self.mass
        ay = Fy / self.mass

        self.acceleration = np.array([ax, ay, 0.0])
        self.velocity += self.acceleration * dt
        self.position += self.velocity * dt
        self.yaw += steer * dt  # 단순 조향 반영 (실제는 속도/조향각 기반 yaw rate 필요)

    # 실제 시뮬레이션 연동 시, 차량 상태/입력/출력 구조를 확장할 수 있음 