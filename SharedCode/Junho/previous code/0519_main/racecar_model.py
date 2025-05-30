"""
목적 : 차량·지형 모델 (서스펜션 + 조향 안정성 강화)
작성자 : 박진석
최초 작성일자 : 2025-05-03
수정자 : 김준호
최종 수정일자 : 2025‑05‑17
"""

import mujoco
import os

XML = """
<!--
목적 : (차량 모델 분리) 부분 트럭 모델
작성자 : 
최초 작성일자 : 2025-05-10
수정자 : 
최종 수정일자 : 2025-05-10
-->
<mujoco>
  <compiler inertiafromgeom="true"/>
  <option timestep="0.001" gravity="0 0 -9.81"/>

  <asset>
    <texture name="grid" type="2d" builtin="checker"
             rgb1="0.2 0.3 0.4" rgb2="0.8 0.8 0.8"
             width="512" height="512"/>
    <material name="grid_mat" texture="grid" texrepeat="5 5"
              specular="0.3" shininess="0.3"/>
    <texture name="wheel_pattern" type="2d" builtin="checker"
             rgb1="1 1 1" rgb2="0 0 0"
             width="64" height="64"/>
    <material name="wheel_mat" texture="wheel_pattern" texrepeat="8 1"
              specular="0.5" shininess="0.5"/>
    <mesh name="VAN_Front_mesh" file="VAN_Front.stl" scale="0.025 0.02 0.025" maxhullvert="100"/>
  </asset>

  <worldbody>
    <light directional="true" diffuse=".8 .8 .8" specular="0.2 0.2 0.2" pos="0 0 4" dir="0 0 -1"/>

    <!-- 바닥: 충돌용 평면 -->
    <geom name="floor" type="plane" size="100 100 0.1"
          material="grid_mat"
          friction="1.0 0.02 0.005"
          contype="1" conaffinity="1" condim="3"/>

    <!-- 차량 본체 + 바퀴 정의 -->
    <body name="chassis" pos="0 0 .5" euler="0 0 0">
      <joint name="root" type="free"/>
      <geom name="chassis" type="box" size="2.7 1.05 0.04" pos="-0.7 0 0" rgba="0.5 0.2 0.7 1" mass="1500" friction="1 0.5 0.5"/>
      <geom name="weight" type="box" size=".1 .1 .1" pos="-1.7 0 0" rgba="1 0.1 0.1 1" mass="1000" friction="1 0.5 0.5"/>          
      <geom name="chassis2" type="box" size="2.7 1.05 0.04" pos="-0.7 0 0.5" rgba="0.5 0.2 0.7 1" mass="1500" friction="1 0.5 0.5"/>
      <geom name="wall_Left" type="box" size="2.7 0.04 1.2" pos="-.7 1 1.2" rgba="0.8 0.8 0.8 0.7" mass="100" friction="1 0.5 0.5"/>
      <geom name="wall_Right" type="box" size="2.7 0.04 1.2" pos="-.7 -1 1.2" rgba="0.8 0.8 0.8 0.7" mass="100" friction="1 0.5 0.5"/>  
      <geom name="wall_Back" type="box" size="0.04 1.05 1.2" pos="-3.35 0 1.2" rgba="0.2 0.2 0.2 0.3" mass="100" friction="1 0.5 0.5"/>
      <geom name="wall_Front" type="box" size="0.04 1.05 1.2" pos="1.9 0 1.2" rgba="0.8 0.8 0.8 1" mass="100" friction="1 0.5 0.5"/>
      <geom name="ceiling" type="box" size="2.7 1.05 0.04" pos="-0.7 0 2.4" rgba="0.5 0.2 0.7 1" mass="100" friction="1 0.5 0.5"/>
      <geom name="VAN_Front" type="mesh" mesh="VAN_Front_mesh" pos="2.6 0 0.2" euler="0 0 -90" rgba="0.5 0.2 0.7 0.8" mass="500" friction="1 0.5 0.5"/>

      <!-- 앞왼쪽 바퀴 -->
      <body name="front_left_susp" pos="2.5 1.5 0.5">
        <inertial pos="0 0 0" mass="5" diaginertia="0.1 0.1 0.1"/>
        <joint name="fl_susp" type="slide" axis="0 0 1"
                range="-0.15 0.05" ref="0.05"
                stiffness="200000" damping="8000"/>
        <body name="front_left_wheel" pos="0 0 -.5">
          <joint name="fl_steer" type="hinge" axis="0 0 1" range="-0.6 0.6"/>
          <joint name="fl_wheel" type="hinge" axis="0 1 0"/>
          <geom name="fl_geom" type="cylinder" size="0.540 0.300"
                euler="90 0 0" material="wheel_mat"
                mass="30" friction="2.0 0.02 0.005"
                solref="0.02 1" solimp="0.9 0.95 0.01 0.5 2"
                contype="1" conaffinity="1" condim="3"
                margin="0.001" gap="0.0001"/>
          <site name="fl_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>

      <!-- 앞오른쪽 바퀴 -->
      <body name="front_right_susp" pos="2.5 -1.5 0.5">
        <inertial pos="0 0 0" mass="5" diaginertia="0.1 0.1 0.1"/>
        <joint name="fr_susp" type="slide" axis="0 0 1"
                range="-0.15 0.05" ref="0.05"
                stiffness="200000" damping="8000"/>
        <body name="front_right_wheel" pos="0 0 -.5">
          <joint name="fr_steer" type="hinge" axis="0 0 1" range="-0.6 0.6"/>
          <joint name="fr_wheel" type="hinge" axis="0 1 0"/>
          <geom name="fr_geom" type="cylinder" size="0.540 0.300"
                euler="90 0 0" material="wheel_mat"
                mass="30" friction="2.0 0.02 0.005"
                solref="0.02 1" solimp="0.9 0.95 0.01 0.5 2"
                contype="1" conaffinity="1" condim="3"
                margin="0.001" gap="0.0001"/>
          <site name="fr_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>

      <!-- 뒤왼쪽 바퀴 -->
      <body name="rear_left_susp" pos="-1.7 1.5 0.5">
        <inertial pos="0 0 0" mass="5" diaginertia="0.1 0.1 0.1"/>
        <joint name="rl_susp" type="slide" axis="0 0 1"
              range="-0.15 0.05" ref="0.05"
              stiffness="300000" damping="8000"/>
        <body name="rear_left_wheel" pos="0 0 -.5">
          <joint name="rl_wheel" type="hinge" axis="0 1 0"/>
          <geom name="rl_geom" type="cylinder" size="0.540 0.300"
              euler="90 0 0" material="wheel_mat"
              mass="30" friction="2.0 0.02 0.005"
              solref="0.02 1" solimp="0.9 0.95 0.01 0.5 2"
              contype="1" conaffinity="1" condim="3"
              margin="0.001" gap="0.0001"/>
          <site name="rl_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>


      <!-- 뒤오른쪽 바퀴 -->
      <body name="rear_right_susp" pos="-1.7 -1.5 0.5">
        <inertial pos="0 0 0" mass="5" diaginertia="0.1 0.1 0.1"/>
        <joint name="rr_susp" type="slide" axis="0 0 1"
              range="-0.15 0.05" ref="0.05"
              stiffness="300000" damping="8000"/>
        <body name="rear_right_wheel" pos="0 0 -.5">
          <joint name="rr_wheel" type="hinge" axis="0 1 0"/>
          <geom name="rr_geom" type="cylinder" size="0.540 0.300"
              euler="90 0 0" material="wheel_mat"
              mass="30" friction="2.0 0.02 0.005"
              solref="0.02 1" solimp="0.9 0.95 0.01 0.5 2"
              contype="1" conaffinity="1" condim="3"
              margin="0.001" gap="0.0001"/>
          <site name="rr_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>
    </body>

    <!-- 기존 점프대 및 장애물 -->
    <geom name="obstacle" type="box" pos="0 10 0"
          size="1 1 2" rgba="0.3 0.8 0.3 1" friction="1 0.5 0.5"
          contype="1" conaffinity="1" condim="3"/>

    <geom name="jump_ramp1" type="box" pos="-20 0 0.1"
          size="1 1 0.1" euler="0 15 0"
          rgba="0.8 0.5 0.3 1" friction="1 0.5 0.5"
          contype="1" conaffinity="1" condim="3"/>
    <geom name="jump_ramp2" type="box" pos="10 10 0.1"
          size="1 1 0.1" euler="0 15 0"
          rgba="0.8 0.5 0.3 1" friction="1 0.5 0.5"
          contype="1" conaffinity="1" condim="3"/>

    <geom name="obstacle1" type="box" pos="-50 -5 0.2"
          size="0.5 0.5 0.2" rgba="0.3 0.8 0.3 1" friction="1 0.5 0.5"
          contype="1" conaffinity="1" condim="3"/>
    <geom name="obstacle2" type="box" pos="-5 40 0.2"
          size="0.5 0.5 0.2" rgba="0.3 0.8 0.3 1" friction="1 0.5 0.5"
          contype="1" conaffinity="1" condim="3"/>
    <geom name="obstacle3" type="box" pos="10 10 0.2"
          size="0.5 0.5 0.2" rgba="1 0 0 1" friction="1 0.5 0.5"
          contype="1" conaffinity="1" condim="3"/>
    <geom name="obstacle4" type="box" pos="10 -10 0.2"
          size="0.5 0.5 0.2" rgba="0.3 0.8 0.3 1" friction="1 0.5 0.5"
          contype="1" conaffinity="1" condim="3"/>
    <geom name="obstacle5" type="box" pos="-10 10 0.2"
          size="0.5 0.5 0.2" rgba="0.3 0.8 0.3 1" friction="1 0.5 0.5"
          contype="1" conaffinity="1" condim="3"/>
    <geom name="obstacle6" type="box" pos="-10 -10 0.2"
          size="0.5 0.5 0.2" rgba="0.3 0.8 0.3 1" friction="1 0.5 0.5"
          contype="1" conaffinity="1" condim="3"/>

    <!-- 수화물 공 9개 -->
    <body name="sphere_1" pos="-1 -.5 2">
      <joint name="sphere_rot_1" type="free"/>
      <geom size="0.1 0.1 0.1" type="sphere" rgba="0.1 0.9 0.2 1" mass="5" contype="1" conaffinity="1" friction="0.04 0.04 0.04"/> 
    </body>
    <body name="sphere_2" pos="0 -.5 2">
      <joint name="sphere_rot_2" type="free"/>
      <geom size="0.1 0.1 0.1" type="sphere" rgba="0.1 0.9 0.2 1" mass="5" contype="1" conaffinity="1" friction="0.04 0.04 0.04"/> 
    </body>
    <body name="sphere_3" pos="1 -.5 2">
      <joint name="sphere_rot_3" type="free"/>
      <geom size="0.1 0.1 0.1" type="sphere" rgba="0.1 0.9 0.2 1" mass="5" contype="1" conaffinity="1" friction="0.04 0.04 0.04"/> 
    </body>
    <body name="sphere_4" pos="-1 0 2">
      <joint name="sphere_rot_4" type="free"/>
      <geom size="0.1 0.1 0.1" type="sphere" rgba="0.1 0.9 0.2 1" mass="5" contype="1" conaffinity="1" friction="0.04 0.04 0.04"/> 
    </body>
    <body name="sphere_5" pos="0 0 2">
      <joint name="sphere_rot_5" type="free"/>
      <geom size="0.1 0.1 0.1" type="sphere" rgba="0.1 0.9 0.2 1" mass="5" contype="1" conaffinity="1" friction="0.04 0.04 0.04"/> 
    </body>
    <body name="sphere_6" pos="1 0 2">
      <joint name="sphere_rot_6" type="free"/>
      <geom size="0.1 0.1 0.1" type="sphere" rgba="0.1 0.9 0.2 1" mass="5" contype="1" conaffinity="1" friction="0.04 0.04 0.04"/> 
    </body>
    <body name="sphere_7" pos="-1 .5 2">
      <joint name="sphere_rot_7" type="free"/>
      <geom size="0.1 0.1 0.1" type="sphere" rgba="0.1 0.9 0.2 1" mass="5" contype="1" conaffinity="1" friction="0.04 0.04 0.04"/> 
    </body>
    <body name="sphere_8" pos="0 .5 2">
      <joint name="sphere_rot_8" type="free"/>
      <geom size="0.1 0.1 0.1" type="sphere" rgba="0.1 0.9 0.2 1" mass="5" contype="1" conaffinity="1" friction="0.04 0.04 0.04"/> 
    </body>
    <body name="sphere_9" pos="1 .5 2">
      <joint name="sphere_rot_9" type="free"/>
      <geom size="0.1 0.1 0.1" type="sphere" rgba="0.1 0.9 0.2 1" mass="5" contype="1" conaffinity="1" friction="0.04 0.04 0.04"/> 
    </body>
  </worldbody>

  <!-- 액츄에이터 (바퀴 동력 및 조향) -->
  <actuator>
    <!-- 구동 -->
    <motor name="rl_drive" joint="rl_wheel" ctrlrange="-2000 2000" gear="6"/>   <!--권장토크 4천~5천 2000*3 = 6000-->
    <motor name="rr_drive" joint="rr_wheel" ctrlrange="-2000 2000" gear="6"/>
    <!-- 브레이크 -->
    <motor name="rl_brake" joint="rl_wheel" ctrlrange="-3000 3000" gear="5"/>
    <motor name="rr_brake" joint="rr_wheel" ctrlrange="-3000 3000" gear="5"/>
    <motor name="fl_brake" joint="fl_wheel" ctrlrange="-3000 3000" gear="5"/>
    <motor name="fr_brake" joint="fr_wheel" ctrlrange="-3000 3000" gear="5"/>
    <!-- 조향 -->
    <motor name="fl_steer_motor" joint="fl_steer" ctrlrange="-200 200" gear="40"/>   <!--권장토크 2천 200*20 = 4000-->
    <motor name="fr_steer_motor" joint="fr_steer" ctrlrange="-200 200" gear="40"/>
  </actuator>

  <!-- 힘 측정 센서-->
  <sensor>
    <force name="fl_force" site="fl_site"/>
    <force name="fr_force" site="fr_site"/>
    <force name="rl_force" site="rl_site"/>
    <force name="rr_force" site="rr_site"/>
    <jointvel name="fl_angular_velocity_sensor" joint="fl_wheel"/>
    <jointvel name="fr_angular_velocity_sensor" joint="fr_wheel"/>
    <jointvel name="rl_angular_velocity_sensor" joint="rl_wheel"/>
    <jointvel name="rr_angular_velocity_sensor" joint="rr_wheel"/>
  </sensor>
  
</mujoco>
"""

def load_model():
    ASSETS = dict()
    stl_path = 'VAN_Front.stl'  # STL 파일의 상대 경로
    try:
        # 현재 스크립트의 디렉토리를 기준으로 STL 파일 경로 설정
        script_dir = os.path.dirname(os.path.abspath(__file__))
        full_stl_path = os.path.join(script_dir, stl_path)

        with open(full_stl_path, 'rb') as f:
            ASSETS['VAN_Front.stl'] = f.read()
    except FileNotFoundError:
        print("Error: STL 파일을 찾을 수 없습니다.")

    if 'VAN_Front.stl' in ASSETS:
        model = mujoco.MjModel.from_xml_string(XML, ASSETS)
        data = mujoco.MjData(model)
        return model, data

    else:
        print("모델 로드 실패: 에셋이 준비되지 않았습니다.")
