"""
목표 : 입력(조이스틱, 키보드 등) 관리
최초 작성자 : 박도솔
최초 작성일자 : 2025-05-14

기여자 : 박도솔

최종 수정자 : 박도솔
최종 수정일자 : 2025-05-14

참고 : 입력 장치 통합 관리
"""
# InputManager.py
class ControlState:
    def __init__(self, throttle=0.0, brake=0.0, steer=0.0, parking_brake=False,
                 speed_tier=1, mode='Manual', hud_visible=True, reset_requested=False):
        self.throttle = throttle
        self.brake = brake
        self.steer = steer
        self.parking_brake = parking_brake
        self.speed_tier = speed_tier
        self.mode = mode
        self.hud_visible = hud_visible
        self.reset_requested = reset_requested

    def __str__(self):
        return (f"ControlState(throttle={self.throttle}, brake={self.brake}, steer={self.steer}, "
                f"parking_brake={self.parking_brake}, speed_tier={self.speed_tier}, "
                f"mode={self.mode}, hud_visible={self.hud_visible}, reset_requested={self.reset_requested})")

    @classmethod
    def from_dict(cls, d):
        return cls(
            throttle=d.get('throttle', 0.0),
            brake=d.get('brake', 0.0),
            steer=d.get('steer', 0.0),
            parking_brake=d.get('parking_brake', False),
            speed_tier=d.get('speed_tier', 1),
            mode=d.get('mode', 'Manual'),
            hud_visible=d.get('hud_visible', True),
            reset_requested=d.get('reset_requested', False)
        )

class InputManager:
    def __init__(self):
        self.control_state = ControlState()

    def receive_command(self, command_dict):
        # 외부에서 명령(dict) 수신 시 호출
        self.control_state = ControlState.from_dict(command_dict)

    def get_control_state(self):
        return self.control_state