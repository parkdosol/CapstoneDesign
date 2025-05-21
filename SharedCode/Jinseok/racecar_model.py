# racecar_model.py
"""
목적 : 차량 모델
작성자 : 
최초 작성일자 : 2025-05-03
수정자 : 박진석
최종 수정일자 : 2025-05-21
"""


import os
import mujoco

# ──────────────────────────────────────────
# 1. 파라미터 블록 ― 여기 숫자만 고치면 XML이 바로 반영됩니다
TIMESTEP      = 0.005      # 시뮬레이션 스텝 (s)
GRAVITY_Z     = -9.81      # 중력 가속도 (m/s²)
INTEGRATOR    = "RK4"      # Euler / RK4 등

MASS_CHASSIS  = 1000       # 프레임 앞/뒤 (kg)
MASS_ROOF     =  100       # 지붕
MASS_FRONT    =  500       # STL 메쉬 전면

# 서스펜션 아암 질량·관성
MASS_SUSP     = 120        # kg
INERTIA_SUSP  = 50         # kg·m²

# 바퀴
MASS_WHEEL    = 80         # kg
WHEEL_RADIUS  = 0.54/2     # m (지오메트리 size 첫 값)

# 서스펜션
SPRING_K_F    = 200_000    # 전륜 슬라이드 강성 (N/m)
SPRING_K_R    = 300_000    # 후륜 슬라이드 강성
DAMP_SUSP     =   8_000    # 서스펜션 감쇠 (N·s/m)

# 액추에이터 기어비 (토크 전환비)
GEAR_DRIVE    = 6
GEAR_STEER    = 40
GEAR_BRAKE    = 1

# STL 파일명
STL_NAME      = "VAN_Front.stl"
# ──────────────────────────────────────────

# 2. XML 템플릿 --------------------------------------------------------------
XML_TEMPLATE = fr"""
<mujoco model="truck">
  <compiler inertiafromgeom="true"/>
  <option timestep="{TIMESTEP}" gravity="0 0 {GRAVITY_Z}" integrator="{INTEGRATOR}"/>

  <!-- ========= 에셋 ========= -->
  <asset>
    <texture name="grid" type="2d" builtin="checker"
             rgb1=".2 .3 .4" rgb2=".8 .8 .8" width="512" height="512"/>
    <material name="grid_mat" texture="grid" texrepeat="5 5"
              specular=".3" shininess=".3"/>

    <texture name="wheel_pat" type="2d" builtin="checker"
             rgb1="1 1 1" rgb2="0 0 0" width="64" height="64"/>
    <material name="wheel_mat" texture="wheel_pat" texrepeat="8 1"
              specular=".5" shininess=".5"/>

    <mesh name="VAN_Front_mesh" file="{STL_NAME}"
          scale=".025 .02 .025" maxhullvert="100"/>
  </asset>

  <!-- ========= 충돌 제외 ========= -->
  <contact>
    <exclude body1="front_left_susp"  body2="chassis"/>
    <exclude body1="front_right_susp" body2="chassis"/>
    <exclude body1="rear_left_susp"   body2="chassis"/>
    <exclude body1="rear_right_susp"  body2="chassis"/>
    <exclude body1="front_left_wheel"  body2="chassis"/>
    <exclude body1="front_right_wheel" body2="chassis"/>
    <exclude body1="rear_left_wheel"   body2="chassis"/>
    <exclude body1="rear_right_wheel"  body2="chassis"/>
  </contact>

  <!-- ========= 월드 ========= -->
  <worldbody>
    <!-- 기본 광원 -->
    <light directional="true" diffuse=".8 .8 .8"
           specular=".2 .2 .2" pos="0 0 4" dir="0 0 -1"/>

    <!-- 바닥 -->
    <geom name="floor" type="plane" size="100 100 .1"
          material="grid_mat" contype="1" conaffinity="1" condim="3"
          friction="1 .02 .005"/>

    <!-- ─── 차체 및 휠 ─── -->
    <body name="chassis" pos="0 0 .5">
      <joint name="root" type="free"/>

      <!-- 메인 프레임 -->
      <geom name="frame1" type="box" size="2.7 1.05 .04" pos="-.7 0 0"
            rgba=".5 .2 .7 1" mass="{MASS_CHASSIS}"/>
      <geom name="frame2" type="box" size="2.7 1.05 .04" pos="-.7 0 .5"
            rgba=".5 .2 .7 1" mass="{MASS_CHASSIS}"/>
      <geom name="ceiling" type="box" size="2.7 1.05 .04" pos="-.7 0 2.4"
            rgba=".5 .2 .7 1" mass="{MASS_ROOF}"/>
      <!-- STL 전면 -->
      <geom name="VAN_Front" type="mesh" mesh="VAN_Front_mesh"
            pos="2.6 0 .2" euler="0 0 -90"
            rgba=".5 .2 .7 .8" mass="{MASS_FRONT}"/>

      <!-- -------- 앞왼쪽 서스펜션 + 휠 -------- -->
      <body name="front_left_susp" pos="2.5 1.5 .5">
        <inertial pos="0 0 0"
                  mass="{MASS_SUSP}"
                  diaginertia="{INERTIA_SUSP} {INERTIA_SUSP} {INERTIA_SUSP}"/>
        <joint name="fl_susp" type="slide" axis="0 0 1"
               range="-.15 .05" ref=".05"
               stiffness="{SPRING_K_F}" damping="{DAMP_SUSP}"/>
        <body name="front_left_wheel" pos="0 0 -.5">
          <joint name="fl_steer" type="hinge" axis="0 0 1" range="-.6 .6"/>
          <joint name="fl_wheel" type="hinge" axis="0 1 0"/>
          <geom name="fl_geom" type="cylinder" size="{WHEEL_RADIUS*2:.3f} {WHEEL_RADIUS:.3f}"
                euler="90 0 0" material="wheel_mat" mass="{MASS_WHEEL}"
                friction="2 .02 .005"/>
          <site name="fl_site" pos="0 0 -{WHEEL_RADIUS*2:.3f}" size=".01"/>
        </body>
      </body>

      <!-- -------- 앞오른쪽 -------- -->
      <body name="front_right_susp" pos="2.5 -1.5 .5">
        <inertial pos="0 0 0"
                  mass="{MASS_SUSP}"
                  diaginertia="{INERTIA_SUSP} {INERTIA_SUSP} {INERTIA_SUSP}"/>
        <joint name="fr_susp" type="slide" axis="0 0 1"
               range="-.15 .05" ref=".05"
               stiffness="{SPRING_K_F}" damping="{DAMP_SUSP}"/>
        <body name="front_right_wheel" pos="0 0 -.5">
          <joint name="fr_steer" type="hinge" axis="0 0 1" range="-.6 .6"/>
          <joint name="fr_wheel" type="hinge" axis="0 1 0"/>
          <geom name="fr_geom" type="cylinder" size="{WHEEL_RADIUS*2:.3f} {WHEEL_RADIUS:.3f}"
                euler="90 0 0" material="wheel_mat" mass="{MASS_WHEEL}"
                friction="2 .02 .005"/>
          <site name="fr_site" pos="0 0 -{WHEEL_RADIUS*2:.3f}" size=".01"/>
        </body>
      </body>

      <!-- -------- 뒤왼쪽 -------- -->
      <body name="rear_left_susp" pos="-1.7 1.5 .5">
        <inertial pos="0 0 0"
                  mass="{MASS_SUSP}"
                  diaginertia="{INERTIA_SUSP} {INERTIA_SUSP} {INERTIA_SUSP}"/>
        <joint name="rl_susp" type="slide" axis="0 0 1"
               range="-.15 .05" ref=".05"
               stiffness="{SPRING_K_R}" damping="{DAMP_SUSP}"/>
        <body name="rear_left_wheel" pos="0 0 -.5">
          <joint name="rl_wheel" type="hinge" axis="0 1 0"/>
          <geom name="rl_geom" type="cylinder" size="{WHEEL_RADIUS*2:.3f} {WHEEL_RADIUS:.3f}"
                euler="90 0 0" material="wheel_mat" mass="{MASS_WHEEL}"
                friction="2 .02 .005"/>
          <site name="rl_site" pos="0 0 -{WHEEL_RADIUS*2:.3f}" size=".01"/>
        </body>
      </body>

      <!-- -------- 뒤오른쪽 -------- -->
      <body name="rear_right_susp" pos="-1.7 -1.5 .5">
        <inertial pos="0 0 0"
                  mass="{MASS_SUSP}"
                  diaginertia="{INERTIA_SUSP} {INERTIA_SUSP} {INERTIA_SUSP}"/>
        <joint name="rr_susp" type="slide" axis="0 0 1"
               range="-.15 .05" ref=".05"
               stiffness="{SPRING_K_R}" damping="{DAMP_SUSP}"/>
        <body name="rear_right_wheel" pos="0 0 -.5">
          <joint name="rr_wheel" type="hinge" axis="0 1 0"/>
          <geom name="rr_geom" type="cylinder" size="{WHEEL_RADIUS*2:.3f} {WHEEL_RADIUS:.3f}"
                euler="90 0 0" material="wheel_mat" mass="{MASS_WHEEL}"
                friction="2 .02 .005"/>
          <site name="rr_site" pos="0 0 -{WHEEL_RADIUS*2:.3f}" size=".01"/>
        </body>
      </body>
    </body> <!-- chassis 끝 -->

    <!-- 점프대 -->
    <geom name="jump_ramp" type="box" pos="-20 0 .1"
          size="1 1 .1" euler="0 15 0" rgba=".8 .5 .3 1"/>
  </worldbody>

  <!-- ========= 액추에이터 ========= -->
  <actuator>
    <!-- 0 – 3 : 구동 모터 -->
    <motor name="fl_motor" joint="fl_wheel" ctrlrange="-100000 100000" gear="{GEAR_DRIVE}"/>
    <motor name="fr_motor" joint="fr_wheel" ctrlrange="-100000 100000" gear="{GEAR_DRIVE}"/>
    <motor name="rl_motor" joint="rl_wheel" ctrlrange="-100000 100000" gear="{GEAR_DRIVE}"/>
    <motor name="rr_motor" joint="rr_wheel" ctrlrange="-100000 100000" gear="{GEAR_DRIVE}"/>

    <!-- 4 – 5 : 조향 -->
    <motor name="fl_steer_motor" joint="fl_steer" ctrlrange="-4000 4000" gear="{GEAR_STEER}"/>
    <motor name="fr_steer_motor" joint="fr_steer" ctrlrange="-4000 4000" gear="{GEAR_STEER}"/>

    <!-- 6 – 9 : 브레이크 (토크 액추에이터) -->
    <motor name="fl_brake_motor" joint="fl_wheel" ctrlrange="-20000 20000" gear="{GEAR_BRAKE}"/>
    <motor name="fr_brake_motor" joint="fr_wheel" ctrlrange="-20000 20000" gear="{GEAR_BRAKE}"/>
    <motor name="rl_brake_motor" joint="rl_wheel" ctrlrange="-20000 20000" gear="{GEAR_BRAKE}"/>
    <motor name="rr_brake_motor" joint="rr_wheel" ctrlrange="-20000 20000" gear="{GEAR_BRAKE}"/>
  </actuator>

  <!-- ========= 센서 ========= -->
  <sensor>
    <force name="fl_force" site="fl_site"/>
    <force name="fr_force" site="fr_site"/>
    <force name="rl_force" site="rl_site"/>
    <force name="rr_force" site="rr_site"/>
  </sensor>
</mujoco>
"""

# 3. 모델 로드 함수 ----------------------------------------------------------
def load_model():
    """XML + STL 에셋을 MuJoCo 모델로 변환하여 반환"""
    asset_dict = {}
    stl_path = os.path.join(os.path.dirname(__file__), STL_NAME)
    with open(stl_path, "rb") as f:
        asset_dict[STL_NAME] = f.read()

    model = mujoco.MjModel.from_xml_string(XML_TEMPLATE, asset_dict)
    data  = mujoco.MjData(model)
    return model, data
