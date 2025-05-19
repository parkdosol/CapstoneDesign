import numpy as np
import mujoco
import glfw

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
        # 차체 ID
        self.chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        # 바퀴 조인트 ID 목록
        self.wheel_jids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in ("fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel")
        ]
        # 브레이크 액추에이터 인덱스 (ctrl[6]~[9])
        self.brake_ctrl_idxs = [6, 7, 8, 9]

        # 공기압 시스템 파라미터
        self.V_res = V_res
        self.V_ch = V_ch
        self.p = p_init
        self.p_min = p_min
        self.p_max = p_max
        self.p_die = p_die

        # 기계적 파라미터
        self.d_dia = d_dia
        self.mu = mu_drum
        self.r_lever = r_lever

        # 실행 중 상태
        self.braking = False
        self.need_pressure_drop = False
        self.prev_space = False
        self.warning_text = ""
        self.current_speed = 0.0
        self.brake_torque = 0.0

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
                # 브레이크 해제 시 actuator 리셋
                for idx in self.brake_ctrl_idxs:
                    # data가 필요하므로 실제 적용은 apply()에서 수행
                    pass

    def apply(self, data: mujoco.MjData, drive_rpm=1000):
        """
        매 스텝 호출
        - braking=False: 공기압 충전 및 브레이크 액추에이터 0
        - braking=True: 엔진 토크 차단, 챔버 압력 감소, 브레이크 토크 계산 및 액추에이터에 할당
        """
        # 1) 현재 속도 계산
        speed_vec = data.qvel[0:3]
        speed_mag = np.linalg.norm(speed_vec)
        self.current_speed = speed_mag

        # 2) 차체 로컬 전방 속도 계산
        Rch = data.xmat[self.chassis_id].reshape(3, 3)
        forward_speed = np.dot(speed_vec, Rch[:, 0])

        if not self.braking:
            # 비제동 시: 공기압 충전 로직
            if speed_mag > 0.5:
                self.p = min(self.p_max, self.p + 0.015 * drive_rpm)
            else:
                # 완전 정지 시 초기 압력 유지
                self.p = min(self.p, self.p_max)
            # 브레이크 actuator 리셋
            for idx in self.brake_ctrl_idxs:
                data.ctrl[idx] = 0.0

        else:
            # 제동 시: 엔진 토크 차단
            data.ctrl[0:4] = [0.0, 0.0, 0.0, 0.0]

            # 정지 임계 속도에서 완전 정지 처리
            if abs(speed_mag) < 0.1 or abs(forward_speed) < 0.1:
                data.qvel[0:3] = [0.0, 0.0, 0.0]
                # 브레이크 actuator 리셋
                for idx in self.brake_ctrl_idxs:
                    data.ctrl[idx] = 0.0
                return

            # 최초 제동 시 공기압 분배
            if self.need_pressure_drop:
                deltaP = self.p * (self.V_ch / (self.V_res + self.V_ch))
                self.p -= deltaP
                self.need_pressure_drop = False

            # 브레이크 토크 계산
            F_n = self.p * np.pi * (self.d_dia / 2) ** 2
            F_fric = F_n * self.mu
            tau = F_fric * self.r_lever
            # 전진/후진 방향에 따른 부호
            if forward_speed > 0:
                brake_torque = -tau
            else:
                brake_torque = +tau
            self.brake_torque = brake_torque

            # 제동용 액추에이터에 토크 할당
            for idx in self.brake_ctrl_idxs:
                data.ctrl[idx] = brake_torque

        # HUD용 경고 텍스트 업데이트
        if self.p <= self.p_die:
            self.warning_text = "Emergency Brake! Air Pressure Too Low!"
        elif self.p <= self.p_min:
            self.warning_text = "Warning: Low Air Pressure!"
        else:
            self.warning_text = ""
