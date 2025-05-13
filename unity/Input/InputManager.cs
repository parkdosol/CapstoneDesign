using UnityEngine;

public class InputManager : MonoBehaviour
{
    public Keyboard keyboard;
    public Joystick joystick;

    // 현재 입력 모드 (예: 키보드/조이스틱)
    public enum InputMode { Keyboard, Joystick }
    public InputMode currentMode = InputMode.Keyboard;

    void Start()
    {
        // 컴포넌트 자동 할당 (필요시)
        if (keyboard == null) keyboard = GetComponent<Keyboard>();
        if (joystick == null) joystick = GetComponent<Joystick>();
    }

    void Update()
    {
        // 입력 모드 전환 및 입력값 처리 로직 구현
        // 예시: if (Input.GetKeyDown(KeyCode.Tab)) { currentMode = ... }
    }
} 