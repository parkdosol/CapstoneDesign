# keyboard_controller.py
"""
목적 : 컨트롤러
작성자 : 박진석
최초 작성일자 : 2025-05-03
수정자 : 박진석
최종 수정일자 : 2025-05-21
"""

"""
키보드 입력 기반 간단 컨트롤러 (정리 버전)
──────────────────────────────────────────
W / S : 전진·후진 토크  (모터 4개에 동일 토크)
A / D : 좌·우 조향 토크 (앞바퀴 2개에 동일 토크)
"""

import glfw

# ───────────── 파라미터 블록 ─────────────
TORQUE_FWD    =  +500.0   # 전진 토크 [N·m]
TORQUE_REV    =  -500.0   # 후진 토크 [N·m]
STEER_LEFT    = +1000.0   # 좌조향 토크 [N·m]
STEER_RIGHT   = -1000.0   # 우조향 토크 [N·m]
# ────────────────────────────────────────

class KeyboardController:
    """GLFW 키 입력을 모터·조향 토크로 변환"""

    def __init__(self):
        self.torque   = 0.0  # 4개 구동 휠 공통 구동 토크
        self.steering = 0.0  # 좌·우 앞바퀴 조향 토크

    # -------------------------------------------------
    def key_callback(self, window, key, scancode, action, mods):
        """키 이벤트 처리"""
        if action in (glfw.PRESS, glfw.REPEAT):
            if key == glfw.KEY_W:
                self.torque = TORQUE_FWD
            elif key == glfw.KEY_S:
                self.torque = TORQUE_REV
            elif key == glfw.KEY_A:
                self.steering = STEER_LEFT
            elif key == glfw.KEY_D:
                self.steering = STEER_RIGHT

        elif action == glfw.RELEASE:
            # 해당 키가 떨어지면 그 축 토크 0
            if key in (glfw.KEY_W, glfw.KEY_S):
                self.torque = 0.0
            if key in (glfw.KEY_A, glfw.KEY_D):
                self.steering = 0.0

    # -------------------------------------------------
    def apply(self, data):
        """MuJoCo data.ctrl 에 토크 적용 (각 스텝마다 호출)"""
        # 구동 토크 → ctrl[2:3] (2개 휠)
        data.ctrl[2] = data.ctrl[3] = self.torque
        # 조향 토크 → ctrl[4], ctrl[5] (앞바퀴 조향)
        data.ctrl[4] = data.ctrl[5] = self.steering
