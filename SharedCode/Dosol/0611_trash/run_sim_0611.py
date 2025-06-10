import mujoco
import mujoco.viewer
import time

def main():
    mjcf_path = "models/simplecar_mjcf.xml" 
    
    try:
        model = mujoco.MjModel.from_xml_path(mjcf_path)
    except Exception as e:
        print(f"오류: MJCF 파일 '{mjcf_path}'를 로드할 수 없습니다. 파일을 확인하거나 경로를 다시 확인해 주세요: {e}")
        return

    data = mujoco.MjData(model)

    # === 액츄에이터 및 센서 이름 확인 부분 제거 ===
    # 이 부분에서 계속 AttributeError가 발생하므로, 동적으로 이름을 가져오는 로직을 제거합니다.
    # 대신, MJCF 파일에 정의된 이름을 코드가 직접 "알고 있다"고 가정하고 사용합니다.
    # 주의: MJCF 파일의 이름이 바뀌면 이 코드도 수동으로 바꿔야 합니다.
    # ===============================================

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0, 0, 0.1]
        viewer.cam.distance = 1.5

        # 액츄에이터가 실제로 모델에 존재하는지 확인하는 초기 로직 (KeyError 방지)
        # 이 부분이 중요합니다. model.actuator("이름")으로 접근하기 전에,
        # 해당 이름의 액츄에이터가 모델에 실제로 있어야 합니다.
        
        # 액츄에이터 이름 리스트 (MJCF 파일에 정의된 이름과 정확히 일치해야 함)
        # 이전에 제공해 주신 MJCF 파일 내용을 기준으로 작성되었습니다.
        desired_actuators = ["fl_motor", "fr_motor", "rl_motor", "rr_motor"]
        
        # 실제로 모델에 존재하는 액츄에이터만 사용할 수 있도록 필터링
        # 이 부분이 KeyError를 방지하는 핵심입니다.
        available_actuator_ids = {}
        for name in desired_actuators:
            try:
                # model.actuator("이름")이 작동하는지 확인
                actuator_id = model.actuator(name).id
                available_actuator_ids[name] = actuator_id
            except KeyError:
                print(f"경고: MJCF 파일에 '{name}' 액츄에이터가 정의되어 있지 않습니다. 제어에서 제외됩니다.")
            except AttributeError:
                # model.actuator() 자체가 없으면 이쪽으로 올 수 있음 (매우 드문 경우)
                print(f"경고: model 객체에 'actuator' 속성이 없습니다. 액츄에이터 제어가 불가능합니다.")
                available_actuator_ids = {} # 모든 액츄에이터 제어 불가
                break # 루프 종료

        # 센서 이름 리스트 (MJCF 파일에 정의된 이름과 정확히 일치해야 함)
        desired_sensors = ["accel_z", "gyro_pitch"]

        available_sensor_adrs = {}
        for name in desired_sensors:
            try:
                # model.sensor("이름")이 작동하는지 확인
                sensor_adr = model.sensor(name).adr
                available_sensor_adrs[name] = sensor_adr
            except KeyError:
                print(f"경고: MJCF 파일에 '{name}' 센서가 정의되어 있지 않습니다. 데이터 읽기에서 제외됩니다.")
            except AttributeError:
                print(f"경고: model 객체에 'sensor' 속성이 없습니다. 센서 데이터 읽기가 불가능합니다.")
                available_sensor_adrs = {} # 모든 센서 읽기 불가
                break


        print(f"사용 가능한 액츄에이터: {list(available_actuator_ids.keys())}")
        print(f"사용 가능한 센서: {list(available_sensor_adrs.keys())}")


        while viewer.is_running():
            torque = 100
            
            # 필터링된 액츄에이터에만 토크 적용
            for name, actuator_id in available_actuator_ids.items():
                data.ctrl[actuator_id] = torque # 모든 바퀴에 같은 토크 적용
            
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

            acc_z = 0.0 
            gyro_pitch = 0.0 
            
            # 필터링된 센서에서만 데이터 읽기
            if "accel_z" in available_sensor_adrs:
                acc_z = data.sensordata[available_sensor_adrs["accel_z"]]
            if "gyro_pitch" in available_sensor_adrs:
                gyro_pitch = data.sensordata[available_sensor_adrs["gyro_pitch"]]
            
            try:
                print(f"time={data.time:.3f}s | acc_z={acc_z.item():.3f} m/s² | pitch_rate={gyro_pitch.item():.3f} rad/s")
            except AttributeError:
                # 센서 데이터가 이미 스칼라 값인 경우 .item()이 필요 없음
                print(f"time={data.time:.3f}s | acc_z={acc_z:.3f} m/s² | pitch_rate={gyro_pitch:.3f} rad/s")

if __name__ == "__main__":
    main()