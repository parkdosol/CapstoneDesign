"""
목적   : Rendering 함수 정의 
최초 작성일자 : 2025‑05‑25
기여자 : 박진석, 박도솔 
수정자 : 박도솔 
최종 수정일자 : 2025‑05‑25

참고 : 물리법칙 팀 작업 편의를 위한 임시 코드 
"""

import glfw
import mujoco

# ***** 시각화·창 설정 *****
WIN_WIDTH      = 1600
WIN_HEIGHT     = 900
VSYNC          = False        # True=60 fps 고정 / False=제한 해제
SHOW_CONTACT   = True         # 접촉 힘·점 시각화 토글

def init_window(width: int, height: int, title: str) -> "glfw._GLFWwindow":
    """GLFW 창 생성 및 OpenGL 컨텍스트 활성화"""
    if not glfw.init():
        raise RuntimeError("GLFW 초기화 실패")
    win = glfw.create_window(width, height, title, None, None)
    if not win:
        glfw.terminate(); raise RuntimeError("윈도우 생성 실패")
    glfw.make_context_current(win)
    glfw.swap_interval(0 if not VSYNC else 1)  # V‑Sync 설정
    return win


def init_viewer(model: mujoco.MjModel):
    """MuJoCo 기본 뷰어(Camera/Scene/Context) 세팅"""
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.azimuth, cam.elevation = 20.0, 90, -20

    opt = mujoco.MjvOption()
    if SHOW_CONTACT:
        opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True

    scn = mujoco.MjvScene(model, maxgeom=1000)
    ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
    return cam, opt, scn, ctx