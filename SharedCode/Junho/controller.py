# controller.py
"""
목적 : 컨트롤러 + 디스크 브레이크 적용
작성자 : 박진석
최초 작성일자 : 2025-05-03
수정자 : 김준호
최종 수정일자 : 2025-05-17
"""

import glfw, numpy as np, mujoco

class KeyboardController:
    # 부호 함수 정의
    def sgn(x):
        """부호 함수 (0보다 크면 1, 작으면 -1, 0이면 0 반환)"""
        return int(x > 0) - int(x < 0)

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.torque   = 0.0
        self.steering = 0.0
        self.braking  = False
        self.brake_gain = 5000.0
        self.air_pressure = 800000  # 초기 공기압 (Pa)
        self.air_pressure_min = 500000  # 최소 공기압 (kPa)
        self.air_pressure_max = 900000  # 최대 공기압 (kPa)
        self.air_pressure_die = 400000  # 정지 공기압 (kPa)
        self.air_pressure_current = self.air_pressure  # 현재 공기압 (kPa)
        self.V_res = 0.06  # 서비스 탱크 60L
        self.V_ch = 0.005  # 챔버 총합 5L
        self.need_pressure_drop = False
        self.prev_space_pressed = False
        self.warning_text = ""

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
        if key == glfw.KEY_SPACE:
            if action == glfw.PRESS and not self.prev_space_pressed:
                 self.braking = True
                 self.need_pressure_drop = True
                 self.prev_space_pressed = True
            elif action == glfw.RELEASE:
                 self.braking = False
                 self.prev_space_pressed = False
        if action in (glfw.PRESS, glfw.REPEAT):
            if key == glfw.KEY_W:
                self.torque = 2000.0
            elif key == glfw.KEY_S:
                self.torque = -2000.0
            elif key == glfw.KEY_A:
                self.steering = 1000.0
            elif key == glfw.KEY_D:
                self.steering = -1000.0
        elif action == glfw.RELEASE:
            if key in (glfw.KEY_W, glfw.KEY_S):
                self.torque = 0.0
            if key in (glfw.KEY_A, glfw.KEY_D):
                self.steering = 0.0
            if key == glfw.KEY_SPACE:
                self.braking = False

    def apply(self, model: mujoco.MjModel, data: mujoco.MjData):
        # 조향
        data.ctrl[self.act_ids['fl_steer']] = self.steering
        data.ctrl[self.act_ids['fr_steer']] = self.steering

        if self.braking:
          if self.need_pressure_drop:
            deltaP = self.air_pressure_current * (self.V_ch / (self.V_res + self.V_ch))
            self.air_pressure_current -= deltaP #브레이크 제동 시 공기압 변화
            self.need_pressure_drop = False
          exit
        else:
            if abs( (data.qvel[0]**2 + data.qvel[1]**2 + data.qvel[2]**2)**0.5 ) > 0: #속도가 0보다 클 때만 공기압이 충전
              rpm = 1000 # rpmdl 1000이라고 가정
              self.air_pressure_current = min(self.air_pressure_max, self.air_pressure_current + 0.015*rpm) #rpm의 0.015배에 비례하여 공기압 충전
            if self.air_pressure_current >= self.air_pressure_max:
              self.air_pressure_current = 800000  # 900kPa에 도달하면 리셋
            self.warning_text = ""  # 경고 텍스트 초기화

        # 경고 메시지 설정
        if self.air_pressure_current <= self.air_pressure_die:
            self.torque = 0
            self.braking = True
            self.warning_text = "Emergency Brake! Air Pressure Too Low!"  # 400kPa 이하
        elif self.air_pressure_current <= self.air_pressure_min:
            self.warning_text = "Warning: Low Air Pressure!"  # 500kPa 이하
        else:
            self.warning_text = ""  # 정상

        if self.braking:
             # 바퀴 angular velocity로 제동 여부 판단
            self.fl_sensor_name = "fl_angular_velocity_sensor"
            self.fl_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, self.fl_sensor_name)
            self.fl_sensor_start_address = model.sensor_adr[self.fl_sensor_id]
            self.fl_sensor_dimension = model.sensor_dim[self.fl_sensor_id]
            self.fl_angular_velocity_value = data.sensordata[self.fl_sensor_start_address]
            if abs(self.fl_angular_velocity_value) < 0.001 :
                data.ctrl[self.act_ids['fl_brake']] = 0
            
            self.fr_sensor_name = "fr_angular_velocity_sensor"
            self.fr_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, self.fr_sensor_name)
            self.fr_sensor_start_address = model.sensor_adr[self.fr_sensor_id]
            self.fr_sensor_dimension = model.sensor_dim[self.fr_sensor_id]
            self.fr_angular_velocity_value = data.sensordata[self.fr_sensor_start_address]
            if abs(self.fr_angular_velocity_value) < 0.001 :
                data.ctrl[self.act_ids['fr_brake']] = 0
            
            self.rl_sensor_name = "rl_angular_velocity_sensor"
            self.rl_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, self.rl_sensor_name)
            self.rl_sensor_start_address = model.sensor_adr[self.rl_sensor_id]
            self.rl_sensor_dimension = model.sensor_dim[self.rl_sensor_id]
            self.rl_angular_velocity_value = data.sensordata[self.rl_sensor_start_address]
            if abs(self.rl_angular_velocity_value) < 0.001 :
                data.ctrl[self.act_ids['rl_brake']] = 0

            self.rr_sensor_name = "rr_angular_velocity_sensor"
            self.rr_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, self.rr_sensor_name)
            self.rr_sensor_start_address = model.sensor_adr[self.rr_sensor_id]
            self.rr_sensor_dimension = model.sensor_dim[self.rr_sensor_id]
            self.rr_angular_velocity_value = data.sensordata[self.rr_sensor_start_address]
            if abs(self.rr_angular_velocity_value) < 0.001 :
                data.ctrl[self.act_ids['rr_brake']] = 0

            qvel = (data.qvel[0]**2 + data.qvel[1]**2 + data.qvel[2]**2)**0.5
            d_dia = 0.2 #다이어프램 유효 직경
            d_dia = 0.2 #다이어프램 유효 직경
            mu_drum = 0.35 #라이닝과 드럼(또는 디스크) 사이 마찰계수
            r_lever = 0.08 #레버 반경
            G = 1 #기계적 이득
            F_n = self.air_pressure_current * np.pi * (d_dia / 2)**2 # 챔버 출력
            F_fric = F_n * mu_drum #마찰력
            F_pad = F_fric * G
            tau_brake = F_pad * r_lever    # 브레이크 토크 계산
            brake_force = tau_brake

            if self.fl_angular_velocity_value > 0:
                brake_torque_fl = -brake_force
            elif self.fl_angular_velocity_value < 0:
                brake_torque_fl = brake_force
            else:
                brake_torque_fl = 0

            if self.fr_angular_velocity_value > 0:
                brake_torque_fr = -brake_force
            elif self.fr_angular_velocity_value < 0:
                brake_torque_fr = brake_force
            else:
                brake_torque_fr = 0

            if self.rl_angular_velocity_value > 0:
                brake_torque_rl = -brake_force
            elif self.rl_angular_velocity_value < 0:
                brake_torque_rl = brake_force
            else:
                brake_torque_rl = 0

            if self.rr_angular_velocity_value > 0:
                brake_torque_rr = -brake_force
            elif self.rr_angular_velocity_value < 0:
                brake_torque_rr = brake_force
            else:
                brake_torque_rr = 0

            sgn = lambda x: int(x > 0) - int(x < 0)
            brake_torque_fl = sgn(brake_torque_fl) * max(abs(brake_torque_fl * abs(qvel)), abs(brake_torque_fl))  # 속도에 비례하여 브레이크 토크 적용
            brake_torque_fr = sgn(brake_torque_fr) * max(abs(brake_torque_fr * abs(qvel)), abs(brake_torque_fr)) 
            brake_torque_rl = sgn(brake_torque_rl) * max(abs(brake_torque_rl * abs(qvel)), abs(brake_torque_rl)) 
            brake_torque_rr = sgn(brake_torque_rr) * max(abs(brake_torque_rr * abs(qvel)), abs(brake_torque_rr)) 

            data.ctrl[self.act_ids['fl_brake']] = brake_torque_fl
            data.ctrl[self.act_ids['fr_brake']] = brake_torque_fr
            data.ctrl[self.act_ids['rl_brake']] = brake_torque_rl
            data.ctrl[self.act_ids['rr_brake']] = brake_torque_rr

           

        else:
            # 일반 주행 모드
            data.ctrl[self.act_ids['rl_drive']] = self.torque
            data.ctrl[self.act_ids['rr_drive']] = self.torque