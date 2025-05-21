# main.py
"""
목적   : 브레이크 기능 포함 차량 시뮬 루프
작성자 : 윤건영
최초 작성일자 : 2025‑05‑03
수정자 : 박진석
최종 수정일자 : 2025‑05‑21
"""

# ───────────────── 파라미터 블록 ─────────────────
# ***** 주행·물성 관련 *****
SIM_SPEED      = 1.0          # 1.0 = 실시간 (0.5 느림, 2.0 두 배 빠름)
UNSPRUNG_MASS  = 150          # kg  (휠+허브 질량)
SPRING_K       = 500_000      # N/m (슬라이드 스프링 강성)
STATIC_SAG     = (UNSPRUNG_MASS * 9.81) / SPRING_K  # 정적 처짐(m)
IDLE_RPM       = 1_000        # 브레이크 로직용 구동축 공회전 rpm

# ***** 시각화·창 설정 *****
WIN_WIDTH      = 1600
WIN_HEIGHT     = 900
VSYNC          = False        # True=60 fps 고정 / False=제한 해제
SHOW_CONTACT   = True         # 접촉 힘·점 시각화 토글

# ***** 동기화 파라미터 *****
MAX_FRAME_DT   = 1 / 30       # 한 프레임 최대 벽시계 간격(초)  (30 fps 기준)
SLEEP_MARGIN   = 0.001        # sleep 보정(초)  – 너무 길게 대기하지 않도록 약간 여유
# ──────────────────────────────────────────────────

import time, traceback

import mujoco, glfw
from racecar_model   import load_model
from controller      import KeyboardController
from pacejka         import Pacejka
from brake_model     import AirBrake
from vehicle_monitor import VehicleMonitor

# --------------------------------------------------
# 창 및 렌더러 초기화
# --------------------------------------------------

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

# --------------------------------------------------
# 메인 실행 루프
# --------------------------------------------------

def main() -> None:
    """프로그램 시작점"""

    # 1) 모델·데이터 로드
    model, data = load_model()

    # 2) 서스펜션 정적 처짐 적용
    for j_name in ("fl_susp", "fr_susp", "rl_susp", "rr_susp"):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j_name)
        data.qpos[jid] = -STATIC_SAG
    mujoco.mj_forward(model, data)  # 초기 상태 강제 계산

    # 3) 시뮬 구성요소 인스턴스
    pace  = Pacejka(model)
    brake = AirBrake(model)
    ctrl  = KeyboardController()
    hud   = VehicleMonitor(model)

    # 4) 창·뷰어 세팅
    win = init_window(WIN_WIDTH, WIN_HEIGHT, "MuJoCo Vehicle")
    def key_all(win, key, sc, act, mods):
        ctrl.key_callback(win, key, sc, act, mods)
        brake.key_callback(win, key, sc, act, mods)
    glfw.set_key_callback(win, key_all)
    cam, opt, scn, ctx = init_viewer(model)

    # 5) 벽시계 기준점 (perf_counter → 고해상도)
    t0_wall = time.perf_counter()

    # 6) 메인 루프
    try:
        while not glfw.window_should_close(win):
            loop_start = time.perf_counter()
            glfw.poll_events()

            # ── 목표 시뮬 시간 계산 ─────────────────────
            wall_elapsed = (loop_start - t0_wall) * SIM_SPEED
            sim_ahead = wall_elapsed - data.time  # >0 → 시뮬이 뒤처짐

            # 필요한 스텝 수 : (시뮬이 뒤처진 시간 / dt)
            dt = model.opt.timestep
            steps_needed = int(max(0.0, sim_ahead) / dt)

            # mj_step 여러 번 호출해 따라잡기
            for _ in range(steps_needed):
                ctrl.apply(data)
                pace.apply(data, brake)
                brake.apply(data, drive_rpm=IDLE_RPM)
                mujoco.mj_step(model, data)

            # 시뮬이 앞서는 경우 잠시 대기해 동기화
            sim_lead = data.time - wall_elapsed  # >0 → 시뮬이 앞섬
            if sim_lead > 0:
                time.sleep(max(0.0, sim_lead / SIM_SPEED - SLEEP_MARGIN))

            # HUD 업데이트 (루프당 1회)
            hud.update(model, data, brake, ctrl)

            # 렌더링 --------------------------------------------------
            fb_w, fb_h = glfw.get_framebuffer_size(win)
            vp = mujoco.MjrRect(0, 0, fb_w, fb_h)
            cam.lookat[:] = [data.qpos[0], data.qpos[1], 1.0]
            mujoco.mjv_updateScene(model, data, opt, None,
                                   cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
            mujoco.mjr_render(vp, scn, ctx)
            mujoco.mjr_overlay(
                mujoco.mjtFontScale.mjFONTSCALE_150, 1, vp,
                hud.text(), "", ctx)
            glfw.swap_buffers(win)

            # 프레임 속도 제한 (MAX_FRAME_DT)
            loop_end = time.perf_counter()
            frame_dt = loop_end - loop_start
            if frame_dt < MAX_FRAME_DT:
                time.sleep(MAX_FRAME_DT - frame_dt)

    except Exception:
        print("[ERROR] 시뮬레이션 중 예외 발생:")
        traceback.print_exc()
    finally:
        glfw.terminate()
        
        # main.py -- VehicleMonitor 호출 전 잠시 넣고 실행
        print("── 휠 조인트 ID /  각속도(rad/s)  ──")
        for lbl, jname in zip(("FL","FR","RL","RR"),
                              ("fl_wheel","fr_wheel","rl_wheel","rr_wheel")):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            print(f"{lbl}: jid={jid:2d}, qvel={data.qvel[jid]:8.3f}")
        print("───────────────────────────────────")



if __name__ == "__main__":
    main()
