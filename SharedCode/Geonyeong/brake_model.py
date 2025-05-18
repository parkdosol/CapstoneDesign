import numpy as np
import glfw
import mujoco

# 부호 함수: x > 0 → 1, x < 0 → -1, x = 0 → 0
def sgn(x):
    return int(x > 0) - int(x < 0)

class AirBrake:
    def __init__(self, model: mujoco.MjModel,
                 V_res=0.06, V_ch=0.005,
                 p_init=800e3, p_min=500e3, p_max=900e3, p_die=400e3,
                 d_dia=0.2, mu_drum=0.35, r_lever=0.08):
        """
        브레이크 모델 초기화
        :param model: MuJoCo 모델
        :param V_res: 서비스 탱크 부피 (m³)
        :param V_ch: 챔버 부피 (m³)
        :param p_init: 초기 공기압 (Pa)
        :param p_min: 저압 경고 임계 (Pa)
        :param p_max: 최대 충전 압력 (Pa)
        :param p_die: 비상 제동 임계 (Pa)
        :param d_dia: 다이어프램 유효 직경 (m)
        :param mu_drum: 마찰계수
        :param r_lever: 브레이크 레버 반경 (m)
        """
        self.model = model
        # Body 및 Wheel joint ID 캐싱
        self.chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self.wheel_jids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in ("fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel")
        ]

        # 공기압 시스템
        self.V_res = V_res
        self.V_ch = V_ch
        self.p_init = p_init
        self.p = p_init
        self.p_min = p_min
        self.p_max = p_max
        self.p_die = p_die

        # 기계적 파라미터
        self.d_dia = d_dia
        self.mu = mu_drum
        self.r_lever = r_lever
        self.wheel_radius = 0.35

        # 내부 상태
        self.brake_torque = 0.0
        self.braking = False
        self.need_pressure_drop = False
        self.prev_space = False
        self.warning_text = ""
        self.current_speed = 0.0

    def key_callback(self, window, key, scancode, action, mods):
        """
        스페이스바 눌림/해제 => 브레이크 상태 토글
        """
        if key == glfw.KEY_SPACE:
            if action == glfw.PRESS and not self.prev_space:
                self.braking = True
                self.need_pressure_drop = True
                self.prev_space = True
            elif action == glfw.RELEASE:
                self.braking = False
                self.prev_space = False
                self.brake_torque = 0.0

    def apply(self, data: mujoco.MjData, drive_rpm=1000):
        """
        매 스텝 호출
        - braking=True: 토크→포스 변환하여 외력 적용
        - braking=False: 공기압 충전
        """
        # 속도 크기 (유클리드 노름)
        speed_vec = data.qvel[0:3]
        speed_mag = np.linalg.norm(speed_vec)
        self.current_speed = speed_mag

        # 로컬 전방 속도 (부호 포함)
        R = data.xmat[self.chassis_id].reshape(3, 3)
        forward_dir = R[:, 0]
        vx, vy, vz = speed_vec
        forward_speed = np.dot([vx, vy, vz], forward_dir)

        if self.braking:
            # 정지 속도 임계
            if abs(speed_mag) < 0.1 or abs(forward_speed) < 0.1:
                data.qvel[0:3] = [0.0, 0.0, 0.0]
                data.xfrc_applied[self.chassis_id, 0] = 0
                self.brake_torque = 0.0
                return
            # 최초 작동 시 압력 강하
            if self.need_pressure_drop:
                deltaP = self.p * (self.V_ch / (self.V_res + self.V_ch))
                self.p -= deltaP
                self.need_pressure_drop = False
            # 브레이크 토크 계산
            F_n = self.p * np.pi * (self.d_dia / 2) ** 2
            F_fric = F_n * self.mu
            tau = F_fric * self.r_lever
            if forward_speed > 0:
                brake_torque = -tau
            elif forward_speed < 0:
                brake_torque = +tau
            else:
                brake_torque = 0.0
            self.brake_torque = brake_torque
            # 토크→포스 변환 및 적용
            brake_force = abs(brake_torque) / self.wheel_radius
            data.xfrc_applied[self.chassis_id, 0] += -sgn(forward_speed) * brake_force
        else:
            # 공기압 충전
            if speed_mag > 0.5:
                self.p = min(self.p_max, self.p + 0.015 * drive_rpm)
            if self.p >= self.p_max:
                self.p = self.p_init

        # HUD 텍스트 업데이트
        if self.p <= self.p_die:
            self.warning_text = "Emergency Brake! Air Pressure Too Low!"
        elif self.p <= self.p_min:
            self.warning_text = "Warning: Low Air Pressure!"
        else:
            self.warning_text = ""
