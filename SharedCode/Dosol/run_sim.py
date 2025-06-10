import mujoco
import mujoco.viewer
import time

def main():
    # model = mujoco.MjModel.from_xml_path("models/car/vehicle_model.xml")
    model = mujoco.MjModel.from_xml_path("models/generated_scene.xml")
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0, 0, 0.1]
        viewer.cam.distance = 1.5

        while viewer.is_running():

            # 기본 주행 (예: 앞바퀴에 일정 토크)
            torque = 100
            data.ctrl[model.actuator("fl_motor").id] = torque
            data.ctrl[model.actuator("fr_motor").id] = torque
            data.ctrl[model.actuator("rl_motor").id] = torque
            data.ctrl[model.actuator("rr_motor").id] = torque

            # 시뮬레이션 1스텝
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

            # 센서 데이터 읽기
            acc_z = data.sensordata[model.sensor("accel_z").adr]
            gyro_pitch = data.sensordata[model.sensor("gyro_pitch").adr]
            print(f"time={data.time:.3f}s | acc_z={acc_z.item():.3f} m/s² | pitch_rate={gyro_pitch.item():.3f} rad/s")


if __name__ == "__main__":
    main()
