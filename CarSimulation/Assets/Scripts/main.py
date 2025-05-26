# main.py
"""
목적 : main loop
최초 작성일자 : 2025-05-14

기여자 : 김준호, 박도솔, 박진석, 윤건영

최종수정자 : 박도솔 
최종 수정일자 : 2025-05-25

[참고] 
- controller, Brake 추가 완료
- Rendering 추가 예정
- Pacejka 추가 예정

"""

import os
import time

from utils.load_data import *
from input.controller import KeyboardController
from physics.PhysicsManager import PhysicsManager
from utils.render import *

TIME_STEP = 0.01

def init_simulation(params) -> tuple:
    model, data = load_model(params)

    chassis_id = get_chassis_id(model)
    wheel_ids = get_wheel_ids(model)

    ctrl = KeyboardController()
    physics = PhysicsManager(model, data, params, ctrl)

    return model, data, physics, ctrl

def init_viewer_system(model, ctrl, physics) -> tuple:
    win = init_window(WIN_WIDTH, WIN_HEIGHT, "MuJoCo Vehicle")

    def key_all(win, key, sc, act, mods):
        ctrl.key_callback(win, key, sc, act, mods)
        # physics.key_callback(win, key, sc, act, mods) # ???

    glfw.set_key_callback(win, key_all)
    cam, opt, scn, ctx = init_viewer(model) 

    return win, cam, opt, scn, ctx

def main() -> None:
    params = load_json(json_path="/workspace/CarSimulation/Assets/Scripts/physics/physics_param.json")
    model, data, physics, ctrl = init_simulation(params)
    win, cam, opt, scn, ctx = init_viewer_system(model, ctrl, physics)

    physics.apply_static_sag(model, data) # 서스펜션 정적 처짐 적용 
    
    try:
        while not glfw.window_should_close(win):
            glfw.poll_events()
            physics.step()
            
            # TODO : 렌더링 추가 
            mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)

            width, height = glfw.get_framebuffer_size(win) 
            # width, height = params["WIN_WIDTH"], params["WIN_HEIGHT"]
            viewport = mujoco.MjrRect(0, 0, width, height)

            mujoco.mjr_render(viewport, scn, ctx)
            glfw.swap_buffers(win) 

    except Exception as e:
        raise RuntimeError(f"Simulation 중 예외 발생: {e}")

    finally:
        glfw.terminate()
        print("Simulation 종료")

if __name__ == '__main__':
    main()