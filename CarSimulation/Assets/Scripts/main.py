"""
목적 : main loop
작성자 : 박도솔
최초 작성일자 : 2025-05-14
참여자 : 박도솔, 박진석
수정자 : 박도솔 
최종 수정일자 : 2025-05-14

참고 : 기본 틀만 잡는 중 
"""

# main.py
import os

from Input.InputManager import InputManager
from physics.PhysicsManager import PhysicsManager
from utils.load_data import load_params_from_json

def main():
    # 파라미터 로드 (physics_param.json 직접 사용)
    param_path = os.path.join(os.path.dirname(__file__), 'physics', 'physics_param.json')
    car_params, force_params, suspension_params = load_params_from_json(param_path)

    # physics 관리자 초기화
    physics_manager = PhysicsManager(car_params, force_params, suspension_params)

    # 예시 입력값 (throttle, brake, steer 등)
    throttle = 1.0
    brake = 0.0
    steer = 0.0

    for step in range(10):  # 10스텝만 예시로 실행
        # 제어값을 시뮬레이션에 적용 (필요시 PhysicsManager에 맞게 수정)
        # 예: physics_manager.apply_control(throttle, brake, steer)
        # 실제 적용 함수명은 PhysicsManager 구현에 따라 다를 수 있음

        # 시뮬레이션 한 스텝 진행
        physics_manager.step()

        # 결과 출력 (필요시 PhysicsManager에서 상태값 받아서 출력)
        print(f"Step {step+1} 완료")

if __name__ == '__main__':
    main()