import mujoco

def convert_urdf_to_mjcf(urdf_path, mjcf_output_path):
    """
    URDF 파일을 MuJoCo MJCF 파일로 변환하여 저장합니다.
    """
    try:
        # 1. URDF 파일을 읽어 문자열로 가져옵니다.
        with open(urdf_path, 'r', encoding='utf-8') as f:
            urdf_string = f.read()

        # 2. MuJoCo 모델 객체를 생성하면서 URDF를 MJCF로 변환합니다.
        #    이때, assets 인자를 통해 메쉬 파일 경로를 명시적으로 전달하는 것이 좋습니다.
        #    URDF에 정의된 메쉬 파일(예: .stl, .obj)이 있다면, 해당 파일들이 올바른 경로에 있어야 합니다.
        #    만약 메쉬가 URDF 파일과 같은 디렉토리에 있다면, assets=None으로 두어도 MuJoCo가 찾아줄 수 있습니다.
        #    복잡한 경로일 경우, assets={'filename': 'absolute_path'} 와 같이 딕셔너리로 제공합니다.
        model = mujoco.MjModel.from_xml_string(urdf_string)

        # 3. 변환된 MuJoCo 모델을 MJCF 파일로 저장합니다.
        #    mujoco.mj_saveLastXML 함수를 사용합니다.
        mujoco.mj_saveLastXML(mjcf_output_path, model)

        print(f"'{urdf_path}' 파일이 성공적으로 '{mjcf_output_path}'로 변환 및 저장되었습니다.")

    except FileNotFoundError:
        print(f"오류: '{urdf_path}' 파일을 찾을 수 없습니다. 경로를 확인해 주세요.")
    except Exception as e:
        print(f"오류: URDF를 MJCF로 변환하는 중 문제가 발생했습니다: {e}")
        print("URDF 파일 내용에 MuJoCo가 해석할 수 없는 문법 오류가 있을 수 있습니다.")

if __name__ == "__main__":
    # 당신의 URDF 파일 경로 (예: 이전에 사용하셨던 generated_scene.xml 내용)
    input_urdf = "models/simplecar.urdf" # 이 파일에 URDF 내용이 저장되어 있어야 합니다.

    # 저장될 MJCF 파일 경로
    output_mjcf = "models/simplecar_mjcf.xml" # MJCF 파일은 일반적으로 .xml 확장자를 사용합니다.

    # 변환 함수 호출
    convert_urdf_to_mjcf(input_urdf, output_mjcf)

    # 변환된 MJCF 파일을 로드하여 시뮬레이션 실행 예시
    try:
        print(f"변환된 MJCF 파일 '{output_mjcf}'를 로드하여 시뮬레이션 시작...")
        model = mujoco.MjModel.from_xml_path(output_mjcf)
        data = mujoco.MjData(model)
        
        # 여기서부터 시뮬레이션 코드를 넣을 수 있습니다.
        # 예시: viewer 띄우기
        import mujoco.viewer
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = [0, 0, 0.1]
            viewer.cam.distance = 1.5
            print("뷰어 시작! 5초 후 종료합니다.")
            time.sleep(5)
            # 여기서는 간단히 5초만 보여주고 종료합니다.
            # 실제 시뮬레이션 로직은 위에 제시된 run_sim.py의 나머지 부분을 참조하세요.

    except Exception as e:
        print(f"변환된 MJCF 파일 로드 또는 시뮬레이션 중 오류 발생: {e}")


convert_urdf_to_mjcf("models/car/car_0611.xml", "models/simplecar_mjcf.xml")