/*
목적 : 상태 표시 UI
작성자 : 박도솔
최초 작성일자 : 2025-05-13
수정자 : 박도솔
최종 수정일자 : 2025-05-13

참고 : 아직 실행 안해봄 
*/

using UnityEngine;
using UnityEngine.UI;

public class StatusDisplay : MonoBehaviour
{
    public InputManager inputManager; // 인스펙터에서 할당
    public Text statusText;           // 인스펙터에서 할당

    void Update()
    {
        if (inputManager == null || statusText == null) return;

        var ctrl = inputManager.CurrentControlState;
        statusText.text =
            $"입력 모드: {inputManager.currentMode}\n" +
            $"Throttle: {ctrl.throttle}\n" +
            $"Brake: {ctrl.brake}\n" +
            $"Steer: {ctrl.steer}\n" +
            $"SpeedTier: {ctrl.speedTier}\n" +
            $"DriveMode: {ctrl.mode}\n" +
            $"ParkingBrake: {ctrl.parkingBrake}\n" +
            $"HUD: {ctrl.hudVisible}\n" +
            $"Reset: {ctrl.resetRequested}";
    }
} 