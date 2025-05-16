"""
목적 : 차량·지형 모델 (서스펜션 + 조향 안정성 강화)
작성자 : 박진석
수정자 : 박진석
최종 수정일자 : 2025‑05‑16
"""

import mujoco

XML = """
<mujoco model="racecar">
  <compiler inertiafromgeom="true"/>
  <option timestep="0.001" gravity="0 0 -9.81"/>

  <!-- ====== 자산 ====== -->
  <asset>
    <texture name="grid" type="2d" builtin="checker"
             rgb1="0.2 0.3 0.4" rgb2="0.8 0.8 0.8"
             width="500" height="500"/>
    <material name="grid_mat" texture="grid" texrepeat="5 5"
              specular="0.3" shininess="0.3"/>
    <texture name="wheel_pattern" type="2d" builtin="checker"
             rgb1="1 1 1" rgb2="0 0 0"
             width="64" height="64"/>
    <material name="wheel_mat" texture="wheel_pattern" texrepeat="8 1"
              specular="0.5" shininess="0.5"/>
  </asset>

  <!-- ====== 월드 ====== -->
  <worldbody>
    <geom name="floor" type="plane" size="500 500 0.1"
          material="grid_mat" friction="1 0.5 0.5"
          contype="1" conaffinity="1" condim="3"/>

    <!-- ====== 차체 ====== -->
    <body name="chassis" pos="0 0 1.5">
      <joint name="root" type="free"/>
      <geom name="chassis" type="box" size="1.875 1.25 0.375"
            rgba="1 0 0 1" mass="3000" friction="1 0.5 0.5"/>
      <geom name="front"  type="box" size="0.625 0.375 0.125"
            pos="1.25 0 0.375" rgba="0 1 0 1" friction="1 0.5 0.5"/>
      <geom name="rear"   type="box" size="0.625 0.375 0.125"
            pos="-1.25 0 0.375" rgba="0 0 1 1" friction="1 0.5 0.5"/>

      <!-- ===== 서스펜션 / 바퀴 4개 ===== -->
      <!-- 앞왼쪽 -->
      <body name="front_left_susp" pos="1.25 1.5 0.7">
        <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
        <joint name="fl_susp" type="slide" axis="0 0 1"
               range="-0.15 0.05" ref="0.05"
               stiffness="20000" damping="3000"/>
        <body name="front_left_wheel" pos="0 0 -1">
          <joint name="fl_steer" type="hinge" axis="0 0 1"
                 range="-0.6 0.6" damping="50" stiffness="1000"/>
          <joint name="fl_wheel" type="hinge" axis="0 1 0" damping="0.1"/>
          <geom name="fl_geom" type="cylinder" size="0.35 0.175"
                euler="90 0 0" material="wheel_mat" mass="7"
                friction="1 0.5 0.5"
                solimp="0.9 0.95 0.001 0.1 2"
                contype="1" conaffinity="1" condim="3"/>
          <site name="fl_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>

      <!-- 앞오른쪽 -->
      <body name="front_right_susp" pos="1.25 -1.5 0.7">
        <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
        <joint name="fr_susp" type="slide" axis="0 0 1"
               range="-0.15 0.05" ref="0.05"
               stiffness="20000" damping="3000"/>
        <body name="front_right_wheel" pos="0 0 -1">
          <joint name="fr_steer" type="hinge" axis="0 0 1"
                 range="-0.6 0.6" damping="50" stiffness="1000"/>
          <joint name="fr_wheel" type="hinge" axis="0 1 0" damping="0.1"/>
          <geom name="fr_geom" type="cylinder" size="0.35 0.175"
                euler="90 0 0" material="wheel_mat" mass="7"
                friction="1 0.5 0.5"
                solimp="0.9 0.95 0.001 0.1 2"
                contype="1" conaffinity="1" condim="3"/>
          <site name="fr_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>

      <!-- 뒤왼쪽 -->
      <body name="rear_left_susp" pos="-1.25 1.5 0.7">
        <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
        <joint name="rl_susp" type="slide" axis="0 0 1"
               range="-0.15 0.05" ref="0.05"
               stiffness="20000" damping="3000"/>
        <body name="rear_left_wheel" pos="0 0 -1">
          <joint name="rl_wheel" type="hinge" axis="0 1 0" damping="0.1"/>
          <geom name="rl_geom" type="cylinder" size="0.35 0.175"
                euler="90 0 0" material="wheel_mat" mass="7"
                friction="1 0.5 0.5"
                solimp="0.9 0.95 0.001 0.1 2"
                contype="1" conaffinity="1" condim="3"/>
          <site name="rl_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>

      <!-- 뒤오른쪽 -->
      <body name="rear_right_susp" pos="-1.25 -1.5 0.7">
        <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
        <joint name="rr_susp" type="slide" axis="0 0 1"
               range="-0.15 0.05" ref="0.05"
               stiffness="20000" damping="3000"/>
        <body name="rear_right_wheel" pos="0 0 -1">
          <joint name="rr_wheel" type="hinge" axis="0 1 0" damping="0.1"/>
          <geom name="rr_geom" type="cylinder" size="0.35 0.175"
                euler="90 0 0" material="wheel_mat" mass="7"
                friction="1 0.5 0.5"
                solimp="0.9 0.95 0.001 0.1 2"
                contype="1" conaffinity="1" condim="3"/>
          <site name="rr_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>
    </body>
  </worldbody>

  <!-- ====== 액추에이터 ====== -->
  <actuator>
    <!-- 구동 -->
    <motor name="rl_drive" joint="rl_wheel" ctrlrange="-2000 2000" gear="1"/>
    <motor name="rr_drive" joint="rr_wheel" ctrlrange="-2000 2000" gear="1"/>

    <!-- 브레이크 -->
    <motor name="rl_brake" joint="rl_wheel" ctrlrange="-3000 3000" gear="1"/>
    <motor name="rr_brake" joint="rr_wheel" ctrlrange="-3000 3000" gear="1"/>
    <motor name="fl_brake" joint="fl_wheel" ctrlrange="-3000 3000" gear="1"/>
    <motor name="fr_brake" joint="fr_wheel" ctrlrange="-3000 3000" gear="1"/>


    <!-- 조향 -->
    <motor name="fl_steer_motor" joint="fl_steer" ctrlrange="-200 200" gear="10"/>
    <motor name="fr_steer_motor" joint="fr_steer" ctrlrange="-200 200" gear="10"/>
  </actuator>

  <!-- ====== 센서 ====== -->
  <sensor>
    <force name="fl_force" site="fl_site"/>
    <force name="fr_force" site="fr_site"/>
    <force name="rl_force" site="rl_site"/>
    <force name="rr_force" site="rr_site"/>
  </sensor>
</mujoco>
"""

def load_model():
    model = mujoco.MjModel.from_xml_string(XML)
    data  = mujoco.MjData(model)
    return model, data
