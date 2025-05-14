"""
목표 : Pacejka 타이어 모델 적용
최초 작성자 : 박도솔
최초 작성일자 : 2025-05-14

기여자 : 박도솔, 박진석

최종 수정자 : 박도솔
최종 수정일자 : 2025-05-14

참고 : 기존 코드 통합 및 xml 파싱 작업중  
"""

import numpy as np
import json
import os
from utils.load_data import load_params_from_json

class PacejkaTireModel:
    """
    Pacejka 타이어 모델을 이용한 타이어 힘 계산 클래스
    """
    def __init__(self, car_params, force_params):
        # 차량 파라미터
        self.m = car_params['mass']
        self.h_cog = car_params.get('h_cog', 0.9)
        self.wbase = car_params['wheelbase'] 
        self.track = car_params['track']
        self.r = car_params['r']
        self.damp = car_params['damp']

        # 차량 힘 파라미터 
        self.Bx, self.Cx, self.Ex, self.mu_x = force_params['Bx'], force_params['Cx'], force_params['Ex'], force_params['mu_x']
        self.By, self.Cy, self.Ey, self.mu_y = force_params['By'], force_params['Cy'], force_params['Ey'], force_params['mu_y']
        self.max_load = force_params['max_load']
        self.max_Fx = self.mu_x * self.max_load
        self.max_Fy = self.mu_y * self.max_load

    @staticmethod
    def magic_formula(slip, Fz, B, C, E, mu, max_D=None):
        D = mu * Fz
        if max_D is not None:
            D = np.minimum(D, max_D)
        Bs = B * slip
        return D * np.sin(C * np.arctan(Bs - E * (Bs - np.arctan(Bs))))

    def compute_forces(self, slip_ratio, slip_angle, Fz):
        """
        slip_ratio: 종방향 슬립 (kappa)
        slip_angle: 횡방향 슬립 (beta)
        Fz: 타이어 하중 (N)
        """
        Fx = float(np.clip(
            self.magic_formula(slip_ratio, Fz, self.Bx, self.Cx, self.Ex, self.mu_x, self.max_Fx),
            -self.max_Fx, self.max_Fx))
        Fy = float(np.clip(
            self.magic_formula(slip_angle, Fz, self.By, self.Cy, self.Ey, self.mu_y, self.max_Fy),
            -self.max_Fy, self.max_Fy))
        return Fx, Fy

    # 실제 차량 데이터와 연동 시, 아래와 같은 메서드로 각 바퀴에 힘을 적용할 수 있음
    # def apply_to_vehicle(self, vehicle_state):
    #     ... 