/*
목적 : Keyboard 조작 코드
작성자 : 박도솔
최초 작성일자 : 2025-05-13
수정자 : 박도솔
최종 수정일자 : 2025-05-13

참고 : README.md 파일의 조작 방법을 참고해주세요.
*/

using UnityEngine;

public class Keyboard : MonoBehaviour
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
    public float throttle;      // 전진/후진 가속
    public float brake;         // 브레이크(후진/감속)
    public float steer;         // 핸들 조향값
    public bool parkingBrake;   // 주차 브레이크
    public int speedTier = 1;   // 0: 저속, 1: 일반, 2: 고속
    public DriveMode mode = DriveMode.Manual;
    public bool hudVisible = true; // HUD 표시 여부
    public bool resetRequested = false; // 차량 리셋 요청

    void Update()
    {
        // --- 기본값 초기화 ---
        throttle = 0f;
        brake = 0f;
        steer = 0f;
        resetRequested = false;

        // --- 운전 조작 ---
        if (Input.GetKey(KeyCode.W))
            throttle = 1f;
        if (Input.GetKey(KeyCode.S))
            brake = 1f;
        if (Input.GetKey(KeyCode.A))
            steer = -1f;
        if (Input.GetKey(KeyCode.D))
            steer = 1f;

        // --- 주차 브레이크 토글 ---
        if (Input.GetKeyDown(KeyCode.Space))
            parkingBrake = !parkingBrake;

        // --- 속도 티어 변경 (Shift) ---
        if (Input.GetKeyDown(KeyCode.LeftShift) || Input.GetKeyDown(KeyCode.RightShift))
        {
            speedTier = (speedTier + 1) % 3; // 0→1→2→0 루프
        }

        // --- 모드 전환 (M) ---
        if (Input.GetKeyDown(KeyCode.M))
        {
            mode = (mode == DriveMode.Manual) ? DriveMode.Auto : DriveMode.Manual;
        }

        // --- HUD 토글 (H) ---
        if (Input.GetKeyDown(KeyCode.H))
        {
            hudVisible = !hudVisible;
        }

        // --- 차량 리셋 (R) ---
        if (Input.GetKeyDown(KeyCode.R))
        {
            resetRequested = true;
        }
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