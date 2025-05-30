import mujoco

# 전체 차량 모델 XML (장애물 추가, 각 바퀴 서스펜션 포함, 충돌 필터링 적용)
XML = """
<mujoco model="racecar">
  <compiler inertiafromgeom="true"/>
  <!-- 시뮬레이션 시간 스텝과 중력 설정 -->
  <option timestep="0.001" gravity="0 0 -9.81"/>

  <asset>
    <!-- 바닥 격자 텍스처 정의 -->
    <texture name="grid" type="2d" builtin="checker"
             rgb1="0.2 0.3 0.4" rgb2="0.8 0.8 0.8"
             width="512" height="512"/>
    <material name="grid_mat" texture="grid" texrepeat="5 5"
              specular="0.3" shininess="0.3"/>
    <!-- 바퀴 패턴 텍스처 정의 -->
    <texture name="wheel_pattern" type="2d" builtin="checker"
             rgb1="1 1 1" rgb2="0 0 0"
             width="64" height="64"/>
    <material name="wheel_mat" texture="wheel_pattern" texrepeat="8 1"
              specular="0.5" shininess="0.5"/>
  </asset>

  <!-- 충돌 필터링: 서스펜션과 차체 간의 접촉 방지 -->
  <contact>
    <exclude body1="fl_susp_body"   body2="chassis"/>
    <exclude body1="fr_susp_body"   body2="chassis"/>
    <exclude body1="rl_susp_body"   body2="chassis"/>
    <exclude body1="rr_susp_body"   body2="chassis"/>
    <exclude body1="front_left_wheel"  body2="chassis"/>
    <exclude body1="front_right_wheel" body2="chassis"/>
    <exclude body1="rear_left_wheel"   body2="chassis"/>
    <exclude body1="rear_right_wheel"  body2="chassis"/>
  </contact>

  <worldbody>
    <!-- 바닥 평면 지오메트리 정의 (물리 충돌용) -->
    <geom name="floor" type="plane" size="50 50 0.1"
          material="grid_mat"
          friction="1 0.5 0.5"
          contype="1" conaffinity="1" condim="3"/>

    <!-- 차체 본체 정의 -->
    <body name="chassis" pos="0 0 0.9">
      <joint name="root" type="free"/>
      <geom name="chassis" type="box" size="1.875 1.25 0.375"
            rgba="1 0 0 1" mass="8000.0" friction="1 0.5 0.5"/>
      <geom name="front" type="box" size="0.625 0.375 0.125"
            pos="1.25 0 0.375" rgba="0 1 0 1" friction="1 0.5 0.5"/>
      <geom name="rear" type="box" size="0.625 0.375 0.125"
            pos="-1.25 0 0.375" rgba="0 0 1 1" friction="1 0.5 0.5"/>

      <!-- 앞왼쪽 서스펜션과 바퀴 정의 -->
      <body name="fl_susp_body" pos="1.25 1.5 0">
        <inertial pos="0 0 0" mass="150" diaginertia="1 1 1"/>
        <joint name="fl_susp" type="slide" axis="0 0 1"
               range="-0.2 0.2" ref="-0.039"
               stiffness="500000" damping="17300"/>
        <body name="front_left_wheel" pos="0 0 -0.35">
          <joint name="fl_steer" type="hinge" axis="0 0 1"
                 range="-0.6 0.6" damping="0" stiffness="0"/>
          <joint name="fl_wheel" type="hinge" axis="0 1 0" damping="0"/>
          <geom name="fl_geom" type="cylinder" size="0.35 0.175"
                euler="90 0 0" material="wheel_mat"
                mass="200.0" friction="1.0 0.1 0.1"
                solimp="0.9 0.95 0.001 0.1 2"
                contype="1" conaffinity="1" condim="3"
                margin="0.001" gap="0.0001"/>
          <site name="fl_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>

      <!-- 앞오른쪽 서스펜션과 바퀴 정의 -->
      <body name="fr_susp_body" pos="1.25 -1.5 0">
        <inertial pos="0 0 0" mass="150" diaginertia="1 1 1"/>
        <joint name="fr_susp" type="slide" axis="0 0 1"
               range="-0.2 0.2" ref="-0.039"
               stiffness="500000" damping="17300"/>
        <body name="front_right_wheel" pos="0 0 -0.35">
          <joint name="fr_steer" type="hinge" axis="0 0 1"
                 range="-0.6 0.6" damping="0" stiffness="0"/>
          <joint name="fr_wheel" type="hinge" axis="0 1 0" damping="0"/>
          <geom name="fr_geom" type="cylinder" size="0.35 0.175"
                euler="90 0 0" material="wheel_mat"
                mass="200.0" friction="1.0 0.1 0.1"
                solimp="0.9 0.95 0.001 0.1 2"
                contype="1" conaffinity="1" condim="3"
                margin="0.001" gap="0.0001"/>
          <site name="fr_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>

      <!-- 뒤왼쪽 서스펜션과 바퀴 정의 -->
      <body name="rl_susp_body" pos="-1.25 1.5 0">
        <inertial pos="0 0 0" mass="150" diaginertia="1 1 1"/>
        <joint name="rl_susp" type="slide" axis="0 0 1"
               range="-0.2 0.2" ref="-0.039"
               stiffness="500000" damping="17300"/>
        <body name="rear_left_wheel" pos="0 0 -0.35">
          <joint name="rl_wheel" type="hinge" axis="0 1 0" damping="0"/>
          <geom name="rl_geom" type="cylinder" size="0.35 0.175"
                euler="90 0 0" material="wheel_mat"
                mass="200.0" friction="1.0 0.1 0.1"
                solimp="0.9 0.95 0.001 0.1 2"
                contype="1" conaffinity="1" condim="3"
                margin="0.001" gap="0.0001"/>
          <site name="rl_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>

      <!-- 뒤오른쪽 서스펜션과 바퀴 정의 -->
      <body name="rr_susp_body" pos="-1.25 -1.5 0">
        <inertial pos="0 0 0" mass="150" diaginertia="1 1 1"/>
        <joint name="rr_susp" type="slide" axis="0 0 1"
               range="-0.2 0.2" ref="-0.039"
               stiffness="500000" damping="17300"/>
        <body name="rear_right_wheel" pos="0 0 -0.35">
          <joint name="rr_wheel" type="hinge" axis="0 1 0" damping="0"/>
          <geom name="rr_geom" type="cylinder" size="0.35 0.175"
                euler="90 0 0" material="wheel_mat"
                mass="200.0" friction="1.0 0.1 0.1"
                solimp="0.9 0.95 0.001 0.1 2"
                contype="1" conaffinity="1" condim="3"
                margin="0.001" gap="0.0001"/>
          <site name="rr_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>

    </body> <!-- chassis 끝 -->

    <!-- 장애물 및 점프대 정의 -->
    <geom name="jump_ramp1" type="box" pos="-20 0 0.1"
          size="1 1 0.1" euler="0 15 0"
          rgba="0.8 0.5 0.3 1" friction="1 0.5 0.5"
          contype="1" conaffinity="1" condim="3"/>
    <geom name="jump_ramp2" type="box" pos="10 1 0.1"
          size="1 1 0.1" euler="0 15 0"
          rgba="0.8 0.5 0.3 1" friction="1 0.5 0.5"
          contype="1" conaffinity="1" condim="3"/>
  </worldbody>

  <actuator>
    <!-- 구동 모터 정의 -->
    <motor name="fl_motor" joint="fl_wheel" ctrlrange="-100000 100000" gear="2"/>
    <motor name="fr_motor" joint="fr_wheel" ctrlrange="-100000 100000" gear="2"/>
    <motor name="rl_motor" joint="rl_wheel" ctrlrange="-100000 100000" gear="2"/>
    <motor name="rr_motor" joint="rr_wheel" ctrlrange="-100000 100000" gear="2"/>
    <!-- 조향 모터 정의 (기어비 50) -->
    <motor name="fl_steer_motor" joint="fl_steer" ctrlrange="-4000 4000" gear="50"/>
    <motor name="fr_steer_motor" joint="fr_steer" ctrlrange="-4000 4000" gear="50"/>
    <!-- 추가: 브레이크 전용 모터 (토크 actuator) -->
    <!-- ctrl[6]~[9]이 네 바퀴 브레이크 토크용이 됩니다 -->
    <motor name="fl_brake_motor" joint="fl_wheel" ctrlrange="-20000 20000" gear="1"/>
    <motor name="fr_brake_motor" joint="fr_wheel" ctrlrange="-20000 20000" gear="1"/>
    <motor name="rl_brake_motor" joint="rl_wheel" ctrlrange="-20000 20000" gear="1"/>
    <motor name="rr_brake_motor" joint="rr_wheel" ctrlrange="-20000 20000" gear="1"/>
  </actuator>

  <sensor>
    <!-- 접촉력 센서 정의 -->
    <force name="fl_force" site="fl_site"/>
    <force name="fr_force" site="fr_site"/>
    <force name="rl_force" site="rl_site"/>
    <force name="rr_force" site="rr_site"/>
  </sensor>
</mujoco>
"""

# 모델 로드 함수
def load_model():
    model = mujoco.MjModel.from_xml_string(XML)
    data  = mujoco.MjData(model)
    return model, data
