import mujoco
import glfw
import traceback
from racecar_model import load_model
from controller import KeyboardController
from pacejka import Pacejka
from brake_model import AirBrake

def init_window(width=1280, height=720, title="MuJoCo Racecar Control"):
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
    # 1) 모델 로드
    model, data = load_model()

    # 2) 정적 처짐(static deflection)으로 qpos 초기화
    #    unsprung mass = 150 kg, spring k = 500000 N/m
    delta_static = (150 * 9.81) / 500000  # ≈0.002943 m
    for name in ("fl_susp", "fr_susp", "rl_susp", "rr_susp"):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[jid] = -delta_static

    # 3) 속도 및 가속도 완전 초기화
    data.qvel[:] = 0
    data.qacc[:] = 0
    
    # 4) 한 번 forward 호출로 내부 상태 갱신
    mujoco.mj_forward(model, data)

    # 5) 컨트롤러 및 뷰어 초기화
    pace  = Pacejka(model)
    brake = AirBrake(model)
    window     = init_window()
    controller = KeyboardController()
    def all_keys(win, key, scancode, action, mods):
        controller.key_callback(win, key, scancode, action, mods)
        brake.key_callback(win, key, scancode, action, mods)
    glfw.set_key_callback(window, all_keys)
    cam, opt, scn, ctx = init_viewer(model)

    try:
        while not glfw.window_should_close(window):
            glfw.poll_events()

            controller.apply(data)
            pace.apply(data, brake)
            brake.apply(data, drive_rpm=1000)
            mujoco.mj_step(model, data)

            cam.lookat[:] = [data.qpos[0], data.qpos[1], 1.0]
            viewport = mujoco.MjrRect(0, 0, 1280, 720)
            mujoco.mjv_updateScene(model, data, opt, None,
                                   cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
            mujoco.mjr_render(viewport, scn, ctx)

            mujoco.mjr_overlay(
                mujoco.mjtFontScale.mjFONTSCALE_150, 0, viewport,
                f"Air Pressure: {int(brake.p/1000)} kPa\nSpeed: {int(brake.current_speed)} m/s",
                brake.warning_text,
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
