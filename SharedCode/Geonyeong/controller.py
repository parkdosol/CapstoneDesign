# controller.py
import glfw

class KeyboardController:
    def __init__(self):
        # 초기 구동 토크, 조향 토크
        self.torque   = 0.0
        self.steering = 0.0

    def key_callback(self, window, key, scancode, action, mods):
        """
        GLFW 키 콜백: W/S → 전후진, A/D → 좌우 조향
        """
        if action in (glfw.PRESS, glfw.REPEAT):
            if key == glfw.KEY_W:
                self.torque =  4000.0
            elif key == glfw.KEY_S:
                self.torque = -4000.0
            elif key == glfw.KEY_A:
                self.steering =  4000.0
            elif key == glfw.KEY_D:
                self.steering = -4000.0

        elif action == glfw.RELEASE:
            # 키 해제 시 해당 제어 입력 리셋
            if key in (glfw.KEY_W, glfw.KEY_S):
                self.torque = 0.0
            if key in (glfw.KEY_A, glfw.KEY_D):
                self.steering = 0.0

    def apply(self, data):
        """
        매 스텝마다 data.ctrl 에 현재 토크/조향 입력을 반영.
        data.ctrl[0:4] = 구동 바퀴
        data.ctrl[4:6] = 조향 바퀴
        """
        data.ctrl[0:4] = [self.torque] * 4
        data.ctrl[4]   = self.steering
        data.ctrl[5]   = self.steering
