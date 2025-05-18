import numpy as np
import mujoco
from brake_model import AirBrake

"""
통합 Pacejka 모델
- 브레이크 토크를 보존하고
- 모든 힘/토크 초기화 → Pacejka 계산 → 브레이크 토크 복원
"""

def magic_formula(slip, Fz, B, C, E, mu, max_D=None):
    """
    Pacejka Magic Formula
      slip  : 슬립비(kappa) 또는 슬립각(beta)
      Fz    : 수직 하중
      B, C, E : Pacejka 계수
      mu    : 마찰계수
      max_D : D(peak값) 최대 허용치 (optional)
    returns: 접지력 (Fx 또는 Fy)
    """
    # D(peak) 계산 및 클램핑
    D = mu * Fz
    if max_D is not None:
        D = np.minimum(D, max_D)
    Bs = B * slip
    return D * np.sin(C * np.arctan(Bs - E * (Bs - np.arctan(Bs))))

class Pacejka:
    """
    Pacejka 모델 클래스
    - 모델에서 body/joint/actuator ID 캐싱
    - apply 호출 시 접지력, 모터 토크 계산
    - 필요 시 브레이크 토크 복원
    """
    def __init__(self, model: mujoco.MjModel):
        self.model = model
        # 휠 정보: (body_name, wheel_joint, steer_joint)
        self.wheels = [
            ('front_left_wheel',  'fl_wheel', 'fl_steer'),
            ('front_right_wheel', 'fr_wheel', 'fr_steer'),
            ('rear_left_wheel',   'rl_wheel', None),
            ('rear_right_wheel',  'rr_wheel', None),
        ]
        # ID 캐싱 (body, joint, steer, actuator)
        self.ids = []
        for body, wj, sj in self.wheels:
            bid   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,     body)
            jid   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,    wj)
            sjid  = (mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, sj)
                     if sj else None)
            aid_name = wj.replace('_wheel', '_motor')
            aid   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aid_name)
            self.ids.append((body, bid, jid, sjid, aid))
        # 차량/물리 파라미터
        cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
        self.m      = float(model.body_mass[cid])         # 전체 질량
        self.h_cog  = float(model.body_pos[cid, 2])      # CoG 높이
        self.wbase  = 2.5                                # 휠베이스
        self.track  = 3.0                                # 트랙 반차폭
        # Pacejka 계수
        self.Bx, self.Cx, self.Ex, self.mu_x = 12.0, 1.9, 0.97, 0.9
        self.By, self.Cy, self.Ey, self.mu_y = 8.5, 1.3, 0.97, 1.0
        self.max_load = 3000.0
        self.max_Fx   = self.mu_x * self.max_load
        self.max_Fy   = self.mu_y * self.max_load
        self.r        = 0.35                            # wheel radius
        self.damp     = 0.1                             # joint damping

    def apply(self, data: mujoco.MjData, brake: AirBrake=None):
        """
        매 스텝 호출
        1) 기존 브레이크 토크 보관
        2) 모든 힘/토크 초기화
        3) Pacejka 접지력, 모터 토크 계산 적용
        4) 브레이크 토크 복원
        """
        # 1) 브레이크 토크 임시 저장
        brake_torques = {}
        if brake and brake.braking:
            for _, _, jid, _, _ in self.ids:
                brake_torques[jid] = float(data.qfrc_applied[jid])

        # 2) 이전 스텝 힘/토크 초기화
        for _, bid, jid, *_ in self.ids:
            data.xfrc_applied[bid] = 0
            data.qfrc_applied[jid] = 0

        # 3) Pacejka 접지력 및 모터 토크 계산
        # 차체 가속도
        ax, ay = float(data.qacc[0]), float(data.qacc[1])
        # 정적 하중
        Fz_stat = self.m * 9.81 / 4.0
        for body, bid, jid, sjid, aid in self.ids:
            # weight transfer 계산
            front = 'front' in body
            left  = 'left'  in body
            dF_long = self.m * ax * self.h_cog / self.wbase
            dF_lat  = self.m * ay * self.h_cog / self.track
            Fz_calc = Fz_stat + 0.5*(dF_long*(1 if front else -1)
                                  + dF_lat*(1 if left else -1))
            Fz = float(np.clip(Fz_calc, 10.0, self.max_load))

            # 바디 로컬 속도
            vel6 = np.zeros(6, float)
            mujoco.mj_objectVelocity(self.model, data,
                                     mujoco.mjtObj.mjOBJ_BODY, bid,
                                     vel6, 1)
            vx, vy = float(vel6[3]), float(vel6[4])

            # wheel spin 및 슬립비 계산
            omega = float(data.qvel[jid])
            vw    = omega * self.r
            vn    = max(abs(vx), 0.5)
            # 정지 마찰(breakaway) 보정
            ctrl_torque = float(data.ctrl[aid])
            if abs(omega) < 1e-3 and abs(ctrl_torque) > 10.0:
                kappa = np.sign(ctrl_torque) * 0.1
            else:
                kappa = float(np.clip((vw - vx) / vn, -0.2, 0.2))

            # 슬립각 계산
            yaw       = float(data.qvel[5])
            dy        = (self.wbase/2 if front else -self.wbase/2)
            steer_ang = float(data.qpos[sjid]) if sjid is not None else 0.0
            beta      = float(np.clip(
                np.arctan2(vy + dy*yaw, vn) - steer_ang,
                -0.26, 0.26))

            # Pacejka force 계산
            Fx = float(np.clip(
                magic_formula(kappa, Fz,
                              self.Bx, self.Cx, self.Ex, self.mu_x,
                              self.max_Fx),
                -self.max_Fx, self.max_Fx))
            Fy = float(np.clip(
                magic_formula(beta,  Fz,
                              self.By, self.Cy, self.Ey, self.mu_y,
                              self.max_Fy),
                -self.max_Fy, self.max_Fy))
            
            # 외력/토크 적용
            data.xfrc_applied[bid, 0] = Fx
            data.xfrc_applied[bid, 1] = Fy
            torque = np.sign(omega) * Fx * self.r + self.damp * omega
            data.qfrc_applied[jid]   -= torque

        # 4) 브레이크 토크 복원
        if brake and brake.braking:
            for jid, tq in brake_torques.items():
                data.qfrc_applied[jid] += tq

