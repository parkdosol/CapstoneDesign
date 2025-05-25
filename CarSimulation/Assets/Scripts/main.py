# main.py
"""
목적 : main loop
최초 작성일자 : 2025-05-14

기여자 : 김준호, 박도솔, 박진석, 윤건영

최종수정자 : 박도솔 
최종 수정일자 : 2025-05-25

참고 : 기본 틀 잡는 중 
"""
import os
import time

from utils.load_data import *
from input.controller import KeyboardController
from physics.PhysicsManager import PhysicsManager
from utils.render import *

TIME_STEP = 0.01

def main() -> None:

    # 파라미터 로드 
    params = load_json()
    model, data = load_model(params)

    # 차체 ID 캐싱 
    chassis_id = get_chassis_id(model)
    wheel_ids = get_wheel_ids(model)

   # 구성 요소 인스턴스 생성 - TODO : HUD 추가 
    physics = PhysicsManager()
    ctrl = KeyboardController()
    
    # 창 뷰어 세팅 
    win = init_window(WIN_WIDTH, WIN_HEIGHT, "MuJoCo Vehicle")
    def key_all(win, key, sc, act, mods):
        ctrl.key_callback(win, key, sc, act, mods)
        physics.key_callback(win, key, sc, act, mods)
    glfw.set_key_callback(win, key_all)
    cam, opt, scn, ctx = init_viewer(model)

    # Main Loop 
    try: 
        while not glfw.window_should_close(win):
            loop_start = time.perf_counter()
            glfw.poll_events()

            ctrl.apply(data) # 입력 처리 
            physics.step(ctrl, dt=TIME_STEP) # 물리 연산 
            

            time.sleep(TIME_STEP)

    except Exception as e:
        raise RuntimeError(f"Simulation 중 예외 발생: {e}")
    
    finally:
        glfw.terminate()
        print("Simulation 종료")

if __name__ == '__main__':
    main()