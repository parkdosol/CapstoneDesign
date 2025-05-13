using UnityEngine;

public class Keyboard : MonoBehaviour
{
    // 차량 제어 변수
    public float throttle;      // 전진/후진 가속
    public float brakeInput;         // 브레이크(후진/감속)
    public float steer;         // 핸들 조향값
    public bool parkingBrake;   // 주차 브레이크
    public int speedTier = 1;   // 0: 저속, 1: 일반, 2: 고속
    public enum DriveMode { Manual, Auto }
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
} 