# main.py
from Input.InputManager import InputManager

def main():
    input_manager = InputManager()

    while True:  # 시뮬레이션 루프
        # 예시: 외부에서 명령을 받아 input_manager에 전달
        # command = receive_from_unity()  # TCP 등으로 명령 수신
        # input_manager.receive_command(command)

        # 현재 입력 상태 가져오기
        ctrl = input_manager.get_control_state()

        print(ctrl)

        # 제어값을 시뮬레이션에 적용
        # 예: vehicle.apply_control(ctrl.throttle, ctrl.brake, ctrl.steer, ...)

        # 시뮬레이션 한 스텝 진행
        # physics_manager.step()

        # 결과 전송 등
        # send_state_to_unity(...)

        # 루프 타이밍 조절
        # time.sleep(dt)

if __name__ == '__main__':
    main()