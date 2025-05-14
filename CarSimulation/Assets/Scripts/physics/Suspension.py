"""
목표 : 서스펜션 모델 적용
최초 작성자 : 박도솔
최초 작성일자 : 2025-05-14

기여자 : 박도솔

최종 수정자 : 박도솔
최종 수정일자 : 2025-05-14

참고 : 임시 생성 코드, 추후 수정 필요
"""

import numpy as np
from utils.load_data import load_params_from_json

class Suspension:
    """
    서스펜션의 스프링/댐퍼 거동을 계산하는 클래스
    """
    def __init__(self, stiffness=20000.0, damping=3000.0, rest_length=0.05, min_length=-0.15, max_length=0.05):
        self.stiffness = stiffness  # N/m
        self.damping = damping      # N·s/m
        self.rest_length = rest_length  # m (기본 위치)
        self.min_length = min_length    # m (최소 변위)
        self.max_length = max_length    # m (최대 변위)

    def compute_force(self, displacement, velocity):
        """
        displacement: 현재 변위 (m)
        velocity: 변위 변화율 (m/s)
        """
        # 스프링 힘: F = -k * (x - x0)
        spring_force = -self.stiffness * (displacement - self.rest_length)
        # 댐퍼 힘: F = -c * v
        damper_force = -self.damping * velocity
        # 총 서스펜션 힘
        total_force = spring_force + damper_force
        return total_force

    def clamp_length(self, length):
        """
        서스펜션 변위가 물리적 한계를 넘지 않도록 제한
        """
        return np.clip(length, self.min_length, self.max_length) 