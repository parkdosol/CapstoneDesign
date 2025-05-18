import mujoco
import glfw
import traceback
from racecar_model import load_model
from controller import KeyboardController
from pacejka import Pacejka
from brake_model import AirBrake

def init_window(width=1280, height=720, title="MuJoCo Racecar Control"):
    """
    GLFW 윈도우 생성 및 컨텍스트 설정
    """
    if not glfw.init():
        raise RuntimeError("GLFW 초기화 실패")
    window = glfw.create_window(width, height, title, None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("윈도우 생성 실패")
    glfw.make_context_current(window)
    return window


def init_viewer(model):
    """
    MuJoCo 카메라/씬/컨텍스트 초기화
    """
    cam = mujoco.MjvCamera()
    cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance  = 20.0
    cam.azimuth   = 90
    cam.elevation = -20

    opt = mujoco.MjvOption()
    # 충돌력/접촉점 시각화
    opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
    opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True

    scn = mujoco.MjvScene(model, maxgeom=1000)
    ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
    return cam, opt, scn, ctx


def main():
    model, data = load_model()
    pace = Pacejka(model)
    brake      = AirBrake(model)

    # 윈도우 & 컨트롤러 초기화
    window     = init_window()
    controller = KeyboardController()
   # 두 컨트롤러의 key_callback을 한데 묶어서 등록
    def all_keys(win, key, scancode, action, mods):
        controller.key_callback(win, key, scancode, action, mods)
        brake.key_callback(win, key, scancode, action, mods)

    glfw.set_key_callback(window, all_keys)

    # 뷰어 초기화
    cam, opt, scn, ctx = init_viewer(model)

    try:
        while not glfw.window_should_close(window):
         glfw.poll_events()

         # 1) 운전 컨트롤,브레이크 로직
         controller.apply(data)
         brake.apply(data, drive_rpm=1000)
         pace.apply(data, brake)

         # 2) 물리 한 스텝
         mujoco.mj_step(model, data)

        
         # 3) 카메라·장면 렌더링
         cam.lookat[:] = [data.qpos[0], data.qpos[1], 1.0]
         viewport = mujoco.MjrRect(0, 0, 1280, 720)
         mujoco.mjv_updateScene(model, data, opt, None,
                               cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
         mujoco.mjr_render(viewport, scn, ctx)

         # 4) HUD
         mujoco.mjr_overlay(
            mujoco.mjtFontScale.mjFONTSCALE_150, 0, viewport,
            f"Air Pressure: {int(brake.p/1000)} kPa\nSpeed: {int(brake.current_speed)} m/s",
            brake.warning_text,
            ctx
        )

         glfw.swap_buffers(window)
    except Exception:
        # 전체 traceback 출력
        print("[ERROR] 시뮬레이션 중 예외 발생:")
        traceback.print_exc()
    finally:
        # 리소스 정리
        glfw.terminate()


if __name__ == "__main__":
    main()