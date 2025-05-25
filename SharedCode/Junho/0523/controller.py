# keyboard_controller.py
"""
목적 : 컨트롤러
작성자 : 박진석
최초 작성일자 : 2025-05-03
수정자 : 김준호
최종 수정일자 : 2025-05-25
"""

"""
키보드 입력 기반 간단 컨트롤러 (정리 버전)
──────────────────────────────────────────
W / S : 전진·후진 토크  (모터 4개에 동일 토크)
A / D : 좌·우 조향 토크 (앞바퀴 2개에 동일 토크)
"""

import glfw

# 3톤 대형트럭의 100km/h 가속 시간은 일반적으로 15-25초 범위
# 단순 계산으로 약 700N·m 정도의 토크가 필요할 것으로 추정.
# 저항 고려시 900 - 1100N·m 추정.
# 안전계수 고려시 1100 - 1500N·m 추정.
# 실제 시뮬레이션 시, 700N·m에서 100km/h 도달까지 약 11초 소요됨.

# 제동시 : 100km/h에서 정지까지 5-7초 소요 추정.
# 4000 - 6000N·m 브레이크 토크가 필요할 것으로 추정. (혹은 5500 - 7500N·m)
# 후륜 바퀴 1개당 1700 - 2300N·m 정도.
# 전륜 바퀴 1개당 2500 - 3500N·m 정도.

# ───────────── 파라미터 블록 ─────────────
TORQUE_FWD    =  +700.0   # 바퀴당 전진 토크 [N·m]
TORQUE_REV    =  -700.0   # 바퀴당 후진 토크 [N·m]
STEER_LEFT    = +2000.0   # 바퀴당 좌조향 토크 [N·m]
STEER_RIGHT   = -2000.0   # 바퀴당 우조향 토크 [N·m]
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
