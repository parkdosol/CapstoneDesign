# main.py
"""
목적 : 메인 코드 
작성자 : 박진석
최초 작성일자 : 2025-05-03
수정자 : 박진석
최종 수정일자 : 2025-05-05
"""

import mujoco, numpy as np
import glfw
import traceback

from racecar_model import load_model
from controller import KeyboardController
from pacejka import Pacejka

def init_window(width=800, height=600, title="MuJoCo Racecar Control"):
    if not glfw.init():
        raise RuntimeError("GLFW 초기화 실패")
    window = glfw.create_window(width, height, title, None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("윈도우 생성 실패")
    glfw.make_context_current(window)
    return window

def init_viewer(model):
    cam = mujoco.MjvCamera()
    cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance  = 20.0
    cam.azimuth   = 90
    cam.elevation = -20

    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
    opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True

    scn = mujoco.MjvScene(model, maxgeom=1000)
    ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
    return cam, opt, scn, ctx

def main():
    model, data = load_model()
    controller = KeyboardController(model)
    pace = Pacejka(model)

    window = init_window()
    glfw.set_key_callback(window, controller.key_callback)

    cam, opt, scn, ctx = init_viewer(model)

    try:
        while not glfw.window_should_close(window):
            glfw.poll_events()
            controller.apply(data)
            pace.apply(data)
            mujoco.mj_step(model, data)

            # 카메라 위치 설정
            cam.lookat[:] = [data.qpos[0], data.qpos[1], 0.9]

            # 렌더링
            viewport = mujoco.MjrRect(0, 0, 800, 600)
            mujoco.mjv_updateScene(model, data, opt, None,
                                   cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
            mujoco.mjr_render(viewport, scn, ctx)

            # ✅ 차체 속도(절대값) km/h 표시
            vx, vy, vz = data.qvel[0:3]            # 루트 바디 선속도 [m/s]
            speed_kmh = np.linalg.norm([vx, vy, vz]) * 3.6   # m/s → km/h
            mujoco.mjr_overlay(
                mujoco.mjtFontScale.mjFONTSCALE_150,
                0, viewport,
                f"Speed: {speed_kmh:.1f} km/h",
                "",
                ctx
            )

            glfw.swap_buffers(window)

    except Exception:
        print("[ERROR] 시뮬레이션 중 예외 발생:")
        traceback.print_exc()
    finally:
        glfw.terminate()

if __name__ == "__main__":
    main()