# controller.py
"""
목적 : 컨트롤러 + 디스크 브레이크 적용
작성자 : 박진석
최초 작성일자 : 2025-05-03
수정자 : 박진석
최종 수정일자 : 2025-05-05
"""

import glfw, numpy as np, mujoco

class KeyboardController:
    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.torque   = 0.0
        self.steering = 0.0
        self.braking  = False
        self.brake_gain = 5000.0

        # actuator ID 캐싱  (앞바퀴 브레이크 추가)
        self.act_ids = {
            'fl_brake': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'fl_brake'),
            'fr_brake': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'fr_brake'),
            'rl_brake': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'rl_brake'),
            'rr_brake': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'rr_brake'),

            'rl_drive': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'rl_drive'),
            'rr_drive': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'rr_drive'),
            'fl_steer': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'fl_steer_motor'),
            'fr_steer': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'fr_steer_motor'),
        }

        # 모든 휠 joint ID  (angular velocity 읽기용)
        self.joint_ids = {
            'fl': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'fl_wheel'),
            'fr': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'fr_wheel'),
            'rl': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'rl_wheel'),
            'rr': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'rr_wheel'),
        }

    def key_callback(self, window, key, scancode, action, mods):
        if action in (glfw.PRESS, glfw.REPEAT):
            if key == glfw.KEY_W:
                self.torque = 2000.0
            elif key == glfw.KEY_S:
                self.torque = -2000.0
            elif key == glfw.KEY_A:
                self.steering = 1000.0
            elif key == glfw.KEY_D:
                self.steering = -1000.0
            elif key == glfw.KEY_SPACE:
                self.braking = True
        elif action == glfw.RELEASE:
            if key in (glfw.KEY_W, glfw.KEY_S):
                self.torque = 0.0
            if key in (glfw.KEY_A, glfw.KEY_D):
                self.steering = 0.0
            if key == glfw.KEY_SPACE:
                self.braking = False

    def apply(self, data: mujoco.MjData):
        # 조향
        data.ctrl[self.act_ids['fl_steer']] = self.steering
        data.ctrl[self.act_ids['fr_steer']] = self.steering

        if self.braking:
            # 구동력 차단
            data.ctrl[self.act_ids['rl_drive']] = 0.0
            data.ctrl[self.act_ids['rr_drive']] = 0.0

            # 네 바퀴 모두 디스크 브레이크
            for wheel in ('fl', 'fr', 'rl', 'rr'):
                omega = data.qvel[self.joint_ids[wheel]]
                data.ctrl[self.act_ids[f'{wheel}_brake']] = -self.brake_gain * omega
        else:
            # 주행 모드
            data.ctrl[self.act_ids['rl_drive']] = self.torque
            data.ctrl[self.act_ids['rr_drive']] = self.torque

            # 브레이크 토크 해제
            for wheel in ('fl', 'fr', 'rl', 'rr'):
                data.ctrl[self.act_ids[f'{wheel}_brake']] = 0.0