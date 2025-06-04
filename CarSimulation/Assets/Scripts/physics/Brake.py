# Brake.py
"""
목적 : 에어 브레이크 + ABS 시스템
기여자 : 김준호, 박도솔, 박진석, 윤건영 
최종 수정자 : 윤건영 
최종 수정일자 : 2025-06-01

[참고]
1. main loop 실행시 여러번 호출하지 않도록 최적화 필요
2. HUD 기능 추가 필요 
"""

import numpy as np
import mujoco
import glfw

from utils.load_data import *

# ────────────────────────────────────────────────────────────────────────────
class AirBrake:
    def __init__(self,model: mujoco.MjModel, params: dict) -> None:
        
        # TODO : main loop 실행시 id 호출 여러번 하지 않도록 최적화 필요

        self.model = model
        self.params = params

        self.cid = get_chassis_id(model)        # 차체 ID
        self.wids = get_wheel_ids(model)        # 바퀴 ID
        self._wheel_jids = [jid for (_, _, jid, _, _) in self.wids]
        self.mass = model.body_mass[self.cid]   # 차체 질량
        self.h_cg = model.body_pos[self.cid, 2] if params["CG_HEIGHT"] is None else params["CG_HEIGHT"] # 차체 중심 높이 
        self._wheel_vel_sensor_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, f"{n}_vel") for n in ("fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel")] # ABS 시스템 (바퀴 각속도 센서)
        
        # 내부 상태 변수
        self.p = params["P_INIT"]
        self.braking = params["BREAK_MODE"]
        self._need_drop = params["P_DROP_MODE"]  # 챔버 압력 드롭 여부 초기화
        self._act_brake = [6,7,8,9]

        self.current_speed = 0.0 #최근 계산 속도 [m/s]
        self.brake_torque = 0.0 #최근 계산 토크 [N·m]
        self.last_torque = [0.0, 0.0, 0.0, 0.0] #각 바퀴별 최근 브레이크 토크 저장
        self.last_pressure = [0.0, 0.0, 0.0, 0.0] #각 바퀴별 최근 압력 [Pa]

    def _compute_vel(self, data: mujoco.MjData) -> float:
       
        v_world = np.array(data.qvel[0:3])
        self.current_speed = float(np.linalg.norm(v_world))
        R = data.xmat[self.cid].reshape(3, 3)
        forward_speed = float(np.dot(v_world, R[:, 0]))
        
        return forward_speed
    
    def _compute_slip(self, data: mujoco.MjData, forward_speed: float, WHEEL_RADIUS: float) -> float:
        
        wheel_speeds = [] # [fl, fr, rl, rr] 각 바퀴별 각속도
        for sid in self._wheel_vel_sensor_ids:
            if sid != -1:                 # 센서가 있을 때만 센서값 읽기
                sensor_adr = self.model.sensor_adr[sid]
                wheel_speeds.append(data.sensordata[sensor_adr])
            else:
                wheel_speeds.append(0.0)  # 센서가 없으면 0으로 처리
        
        slip_ratios = []  # slip_ratios: [fl, fr, rl, rr] 각 바퀴별 슬립비
        for ws in wheel_speeds:
            v_wheel = ws * WHEEL_RADIUS
            denominator = max(abs(forward_speed), 1e-6)
            slip = abs(forward_speed - v_wheel) / denominator
            slip = min(slip, 1.0)
            slip_ratios.append(slip)
        
        return slip_ratios

    # TODO : 여기서부터 정리 해야함 
    
    def _reset_actuator_torque(self, data: mujoco.MjData) -> None:

        for idx in self._act_brake:
            data.ctrl[idx] = 0.0
            self.last_torque[idx] = 0.0

    def _apply_full_stop(self, data: mujoco.MjData):

        data.qvel[0:2] = [0.0, 0.0]
        for jid in self._wheel_jids:
            if jid != -1:
                data.qvel[jid] = 0.0
        self._reset_actuator_torque(data)


    def _estimate_brake_input(self, data: mujoco.MjData, params) -> float:

        if not self.braking: # 비제동 모드

            if self.current_speed > 0.5:
                self.p = min(
                    params["P_MAX"], 
                    self.p 
                    + params["PRESSURE_CHARGE_RATE"] 
                    * params["DRIVE_RPM"]
                )

            self._reset_actuator_torque(data)

        # 제동 모드
        data.ctrl[0:4] = [0.0, 0.0, 0.0, 0.0]

        # 정지 임계속도 이하이면 완전 정지 처리
        if abs(self.current_speed) < params["STOP_THRESH"] :
            self._apply_full_stop(data)
            return
    def _drop_chamber_pressure(self, data: mujoco.MjData,params: dict):
        delta_p = self.p * (
                            params["VOL_CHAMBER"] / (
                            params["VOL_RESERVOIR"] + params["VOL_CHAMBER"])
        )
        self.p -= delta_p
        self._need_drop = False

    def _compute_brake_torque(self, data: mujoco.MjData, params: dict) -> float:
        area = np.pi * (params["DIA_DIAPHRAGM"] / 2) ** 2  # 다이어프램 면적
        F_normal = self.p * area                                           # 챔버 누르는 힘 [N]
        F_fric   = F_normal * params["MU_DRUM"]            # 라이닝 마찰력 [N]
        tau      = F_fric * params["LEVER_RADIUS"]         # 바퀴 축 토크 [N·m]

        return tau

    def _apply_abs_torque(self, data: mujoco.MjData, params: dict) -> None:
        """ABS 로직을 적용해 브레이크 토크를 각 휠에 분배"""
        for i, act_idx in enumerate(self._act_brake):
            wheel_jid = self._wheel_jids[i]
            wheel_vel = data.qvel[wheel_jid]

            # 바퀴가 회전 중이면 회전 방향 반대로 토크를 적용
            if abs(wheel_vel) > 1e-3:
                torque = -np.sign(wheel_vel) * self.brake_torque
            # 거의 정지 상태면 차량 진행 방향 기준으로 토크 적용
            elif abs(self.current_speed) > 1e-3:
                torque = -np.sign(self.current_speed) * self.brake_torque
            else:
                torque = 0.0

            # 슬립비에 따라 ABS 토크 계수 적용
            slip = self.last_slip[i]
            abs_multiplier = params["ABS_TORQUE_LOW"]
            if slip < params["SLIP_MIN"]:
                abs_multiplier = params["ABS_TORQUE_HIGH"]
            elif slip < params["SLIP_MAX"]:
                abs_multiplier = params["ABS_TORQUE_OPTIMAL"]

            data.ctrl[act_idx] = torque * abs_multiplier
            self.last_torque[i] = data.ctrl[act_idx]

    def apply(self, data: mujoco.MjData, params: dict) -> None:
        
        # TODO : main loop 실행시 여러번 호출하지 않도록 최적화 필요 

        self.current_speed = self._compute_vel(data)
        self.last_slip = self._compute_slip(data, self.current_speed, params["WHEEL_DIAMETER"]/2)

        if self._need_drop:
            self._drop_chamber_pressure(data, params)

        self.brake_torque = self._compute_brake_torque(data, params)
        self._apply_abs_torque(data, params)
