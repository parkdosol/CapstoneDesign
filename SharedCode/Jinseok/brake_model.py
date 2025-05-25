# air_brake.py
"""
목적 : 브레이크 기능 추가
작성자 : 윤건영
최초 작성일자 : 2025-05-03
수정자 : 박진석
최종 수정일자 : 2025-05-21
"""


import numpy as np
import mujoco
import glfw

# ───────────────── 파라미터 블록 ─────────────────
# 추가/튜닝이 잦은 값은 아래에서만 수정하면 됩니다.

# 공기압 탱크·챔버 용적 [m³]
VOL_RESERVOIR  = 0.06   # 서비스 탱크 (리저버)
VOL_CHAMBER    = 0.005  # 브레이크 챔버

# 초기·경고·최대 압력 [Pa]
P_INIT = 0.8e6   # 시동 시 초기 압력
P_MIN  = 0.5e6   # 저압 경고 임계값
P_MAX  = 0.9e6   # 리저버 최대 압력(충전 한계)
P_DIE  = 0.4e6   # 비상 제동 임계값 (시뮬 정지)

# 기계적 파라미터
DIA_DIAPHRAGM = 0.20   # 다이어프램 직경 [m]
MU_DRUM       = 0.35   # 라이닝‑드럼 마찰계수
LEVER_RADIUS  = 0.08   # 브레이크 레버 반경 [m]

# 공기압 충전 속도 계수
PRESSURE_CHARGE_RATE = 0.015   # Δp = rate * drive_rpm
# 정지 판정 임계속도 [m/s]
STOP_THRESH = 0.1

# ────────────────────────────────────────────────

class AirBrake:
    """트럭용 에어 브레이크 시뮬레이션 모델"""

    def __init__(self, model: mujoco.MjModel):
        """필요한 ID 캐싱 및 내부 상태 초기화"""
        self.model = model

        # 차체 ID (차체 속도·방향 계산용)
        self._id_chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        # 앞/뒤/좌/우 바퀴 조인트 ID
        self._wheel_jids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
            for n in ("fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel")
        ]
        # 브레이크 토크용 액추에이터(ctrl[6:10]) 인덱스 (XML에 맞춰 고정)
        self._act_brake = [6, 7, 8, 9]

        # ─ 내부 상태 변수 ─
        self.p       = P_INIT   # 현재 리저버 압력 [Pa]
        self.braking = False    # 제동 플래그
        self._need_drop = False # 최초 제동 시 압력 분배 여부
        self._prev_space = False
        self.warning_text = ""  # HUD 경고용 문자열
        self.current_speed = 0.0 # 차체 속도 [m/s]
        self.brake_torque  = 0.0 # 최근 계산 토크 [N·m]

    # ────────────────────────────────────────────
    # 키 입력 처리 --------------------------------------------------------------
    def key_callback(self, window, key, scancode, action, mods):
        """스페이스바로 브레이크 ON/OFF 토글"""
        if key != glfw.KEY_SPACE:
            return
        if action == glfw.PRESS and not self._prev_space:
            self.braking = True
            self._need_drop = True   # 처음 누를 때만 압력 분배
            self._prev_space = True
        elif action == glfw.RELEASE:
            self.braking = False
            self._prev_space = False

    # ────────────────────────────────────────────
    # 메인 로직 ---------------------------------------------------------------
    def apply(self, data: mujoco.MjData, drive_rpm: float = 1000.0):
        """시뮬레이션 매 스텝 호출해 브레이크 로직 수행"""
        # 1) 차체 속도 및 전진 방향 속도 ------------------------------------------------
        v_world = np.array(data.qvel[0:3])
        self.current_speed = float(np.linalg.norm(v_world))
        R = data.xmat[self._id_chassis].reshape(3, 3)
        forward_speed = float(np.dot(v_world, R[:, 0]))

        # 2) 비제동 모드 → 압력 충전 & 브레이크 토크 0 ------------------------------
        if not self.braking:
            if forward_speed > 0.5:   # 주행 중일 때만 압력 서서히 충전
                self.p = min(P_MAX, self.p + PRESSURE_CHARGE_RATE * drive_rpm)
            # 액추에이터 토크 리셋
            for idx in self._act_brake:
                data.ctrl[idx] = 0.0
            self._update_warning_text()
            return

        # ↓↓↓ 제동 중인 경우 ↓↓↓ ---------------------------------------------------
        # 3) 엔진(모터) 토크 차단
        data.ctrl[0:4] = [0.0, 0.0, 0.0, 0.0]

        # 4) 정지 임계속도 이하이면 완전 정지 처리 -------------------------------
        if abs(self.current_speed) < STOP_THRESH or abs(forward_speed) < STOP_THRESH:
            data.qvel[0:3] = [0.0, 0.0, 0.0]
            for idx in self._act_brake:
                data.ctrl[idx] = 0.0
            self._update_warning_text()
            return

        # 5) 최초 제동 시 챔버 압력 강하 ------------------------------------------
        if self._need_drop:
            delta_p = self.p * (VOL_CHAMBER / (VOL_RESERVOIR + VOL_CHAMBER))
            self.p -= delta_p
            self._need_drop = False

        # 6) 브레이크 토크 계산 ----------------------------------------------------
        area = np.pi * (DIA_DIAPHRAGM / 2) ** 2  # 다이어프램 면적
        F_normal = self.p * area                 # 챔버 누르는 힘 [N]
        F_fric   = F_normal * MU_DRUM            # 라이닝 마찰력 [N]
        tau      = F_fric * LEVER_RADIUS         # 바퀴 축 토크 [N·m]
        self.brake_torque = -tau if forward_speed > 0 else tau

        for idx in self._act_brake:
            data.ctrl[idx] = self.brake_torque

        # 7) HUD 경고 업데이트 ------------------------------------------------------
        self._update_warning_text()

    # ────────────────────────────────────────────
    def _update_warning_text(self):
        """리저버 압력 기준으로 HUD 경고 문구 설정"""
        if self.p <= P_DIE:
            self.warning_text = "Emergency Brake! Air Pressure Too Low!"
        elif self.p <= P_MIN:
            self.warning_text = "Warning: Low Air Pressure!"
        else:
            self.warning_text = ""
