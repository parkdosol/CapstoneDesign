/*
목적 : 입력 관리 코드
작성자 : 박도솔
최초 작성일자 : 2025-05-13
수정자 : 박도솔
최종 수정일자 : 2025-05-13

*/

using UnityEngine;

public class InputManager : MonoBehaviour
{
    public Keyboard keyboard;
    public Joystick joystick;

    // 현재 입력 모드 (예: 키보드/조이스틱)
    public enum InputMode { Keyboard, Joystick }
    public InputMode currentMode = InputMode.Keyboard;

    // 현재 입력 상태
    public Keyboard.ControlState CurrentControlState { get; private set; }

    private float logTimer = 0f;
    private const float logInterval = 0.5f; // 0.5초마다 출력

    void Start()
    {
        // 컴포넌트 자동 할당 (필요시)
        if (keyboard == null) keyboard = GetComponent<Keyboard>();
        if (joystick == null) joystick = GetComponent<Joystick>();
    }

    void Update()
    {
        // 조이스틱 연결 여부 확인
        bool joystickConnected = false;
        foreach (var name in Input.GetJoystickNames())
        {
            if (!string.IsNullOrEmpty(name))
            {
                joystickConnected = true;
                break;
            }
        }

        // 입력 모드 자동 전환
        currentMode = joystickConnected ? InputMode.Joystick : InputMode.Keyboard;

        // 현재 입력 상태 갱신
        if (currentMode == InputMode.Joystick)
        {
            CurrentControlState = joystick.GetControlState();
        }
        else
        {
            CurrentControlState = keyboard.GetControlState();
        }

        // 0.5초마다 입력 상태를 콘솔에 출력
        logTimer += Time.deltaTime;
        if (logTimer >= logInterval)
        {
            logTimer = 0f;
            Debug.Log($"[InputManager] Mode: {currentMode} | Throttle: {CurrentControlState.throttle} | Brake: {CurrentControlState.brake} | Steer: {CurrentControlState.steer} | SpeedTier: {CurrentControlState.speedTier} | DriveMode: {CurrentControlState.mode} | ParkingBrake: {CurrentControlState.parkingBrake} | HUD: {CurrentControlState.hudVisible} | Reset: {CurrentControlState.resetRequested}");
        }
    }
} 