/*
목적 : Joystick 조작 코드
작성자 : 박도솔
최초 작성일자 : 2025-05-13
수정자 : 박도솔
최종 수정일자 : 2025-05-13

참고 : PS4 조이스틱을 기준으로 작성되었으며, 아직 실행되지 않은 코드입니다. 

*/

using UnityEngine;

public class Joystick : MonoBehaviour
{
    public enum DriveMode { Manual, Auto }

    public struct ControlState
    {
        public float throttle, brake, steer;
        public bool parkingBrake;
        public int speedTier;
        public DriveMode mode;
        public bool hudVisible;
        public bool resetRequested;
    }

    // 차량 제어 변수
    public float throttle;      // R2 트리거 (전진)
    public float brake;         // L2 트리거 (브레이크/후진)
    public float steer;         // 좌측 스틱 X축 (조향)
    public bool parkingBrake;   // □(Square) 버튼 토글
    public int speedTier = 1;   // △(Triangle) 버튼으로 변경
    public DriveMode mode = DriveMode.Manual; // Options 버튼으로 전환
    public bool hudVisible = true; // Touchpad 버튼으로 토글
    public bool resetRequested = false; // ○(Circle) 버튼으로 리셋

    void Update()
    {
        // --- 기본값 초기화 ---
        throttle = 0f;
        brake = 0f;
        steer = 0f;
        resetRequested = false;

        // --- 입력 처리 (Unity Input Manager 기준) ---
        // R2: "Joystick Axis 5" (1: 미입력, -1: 최대 입력)
        // L2: "Joystick Axis 4" (1: 미입력, -1: 최대 입력)
        // 좌측 스틱 X: "Horizontal"
        // 참고: Input.GetAxis("AxisName") 값 범위는 -1 ~ 1

        // R2 (전진)
        float r2 = (1f - Input.GetAxis("Joystick Axis 5")) / 2f; // 0~1
        throttle = r2;
        // L2 (브레이크)
        float l2 = (1f - Input.GetAxis("Joystick Axis 4")) / 2f; // 0~1
        brake = l2;
        // 좌측 스틱 X (조향)
        steer = Input.GetAxis("Horizontal");

        // □(Square) 버튼: parkingBrake 토글 (joystick button 3)
        if (Input.GetKeyDown(KeyCode.JoystickButton3))
            parkingBrake = !parkingBrake;

        // △(Triangle) 버튼: speedTier 변경 (joystick button 2)
        if (Input.GetKeyDown(KeyCode.JoystickButton2))
            speedTier = (speedTier + 1) % 3;

        // Options 버튼: 모드 전환 (joystick button 9)
        if (Input.GetKeyDown(KeyCode.JoystickButton9))
            mode = (mode == DriveMode.Manual) ? DriveMode.Auto : DriveMode.Manual;

        // Touchpad 버튼: HUD 토글 (joystick button 13, 일부 환경에서는 14)
        if (Input.GetKeyDown(KeyCode.JoystickButton13) || Input.GetKeyDown(KeyCode.JoystickButton14))
            hudVisible = !hudVisible;

        // ○(Circle) 버튼: 차량 리셋 (joystick button 1)
        if (Input.GetKeyDown(KeyCode.JoystickButton1))
            resetRequested = true;
    }

    public ControlState GetControlState()
    {
        return new ControlState
        {
            throttle = this.throttle,
            brake = this.brake,
            steer = this.steer,
            parkingBrake = this.parkingBrake,
            speedTier = this.speedTier,
            mode = this.mode,
            hudVisible = this.hudVisible,
            resetRequested = this.resetRequested
        };
    }
} 