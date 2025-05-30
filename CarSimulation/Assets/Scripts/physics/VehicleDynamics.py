"""
목표 : 차량 물리 코드 통합 관리 코드 
최초 작성자 : 박도솔
최초 작성일자 : 2025-05-14

기여자 : 박도솔

최종 수정자 : 박도솔
최종 수정일자 : 2025-05-25

참고 : 수정 작업중
"""
import numpy as np
import mujoco

class VehicleDynamics:
    """
    위치, 속도, 가속도 등 차량 상태 추적 
    """

    # TODO : 차량 동역학 모듈 추가 
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        # TODO : 내용 추가 
        pass

    def apply(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        # TODO : 내용 추가 
        pass