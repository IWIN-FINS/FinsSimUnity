using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;

public static class InputSystemKeyBridge
{
    public static bool WasPressedThisFrame(KeyCode keyCode)
    {
        Keyboard keyboard = Keyboard.current;
        if (keyboard == null)
        {
            return false;
        }

        KeyControl keyControl = GetKeyControl(keyboard, keyCode);
        return keyControl != null && keyControl.wasPressedThisFrame;
    }

    private static KeyControl GetKeyControl(Keyboard keyboard, KeyCode keyCode)
    {
        switch (keyCode)
        {
            case KeyCode.A: return keyboard.aKey;
            case KeyCode.B: return keyboard.bKey;
            case KeyCode.C: return keyboard.cKey;
            case KeyCode.D: return keyboard.dKey;
            case KeyCode.E: return keyboard.eKey;
            case KeyCode.F: return keyboard.fKey;
            case KeyCode.G: return keyboard.gKey;
            case KeyCode.H: return keyboard.hKey;
            case KeyCode.I: return keyboard.iKey;
            case KeyCode.J: return keyboard.jKey;
            case KeyCode.K: return keyboard.kKey;
            case KeyCode.L: return keyboard.lKey;
            case KeyCode.M: return keyboard.mKey;
            case KeyCode.N: return keyboard.nKey;
            case KeyCode.O: return keyboard.oKey;
            case KeyCode.P: return keyboard.pKey;
            case KeyCode.Q: return keyboard.qKey;
            case KeyCode.R: return keyboard.rKey;
            case KeyCode.S: return keyboard.sKey;
            case KeyCode.T: return keyboard.tKey;
            case KeyCode.U: return keyboard.uKey;
            case KeyCode.V: return keyboard.vKey;
            case KeyCode.W: return keyboard.wKey;
            case KeyCode.X: return keyboard.xKey;
            case KeyCode.Y: return keyboard.yKey;
            case KeyCode.Z: return keyboard.zKey;

            case KeyCode.Alpha0: return keyboard.digit0Key;
            case KeyCode.Alpha1: return keyboard.digit1Key;
            case KeyCode.Alpha2: return keyboard.digit2Key;
            case KeyCode.Alpha3: return keyboard.digit3Key;
            case KeyCode.Alpha4: return keyboard.digit4Key;
            case KeyCode.Alpha5: return keyboard.digit5Key;
            case KeyCode.Alpha6: return keyboard.digit6Key;
            case KeyCode.Alpha7: return keyboard.digit7Key;
            case KeyCode.Alpha8: return keyboard.digit8Key;
            case KeyCode.Alpha9: return keyboard.digit9Key;

            case KeyCode.Space: return keyboard.spaceKey;
            case KeyCode.Tab: return keyboard.tabKey;
            case KeyCode.Return: return keyboard.enterKey;
            case KeyCode.Escape: return keyboard.escapeKey;
            case KeyCode.Backspace: return keyboard.backspaceKey;

            case KeyCode.UpArrow: return keyboard.upArrowKey;
            case KeyCode.DownArrow: return keyboard.downArrowKey;
            case KeyCode.LeftArrow: return keyboard.leftArrowKey;
            case KeyCode.RightArrow: return keyboard.rightArrowKey;

            case KeyCode.LeftShift: return keyboard.leftShiftKey;
            case KeyCode.RightShift: return keyboard.rightShiftKey;
            case KeyCode.LeftControl: return keyboard.leftCtrlKey;
            case KeyCode.RightControl: return keyboard.rightCtrlKey;
            case KeyCode.LeftAlt: return keyboard.leftAltKey;
            case KeyCode.RightAlt: return keyboard.rightAltKey;

            case KeyCode.F1: return keyboard.f1Key;
            case KeyCode.F2: return keyboard.f2Key;
            case KeyCode.F3: return keyboard.f3Key;
            case KeyCode.F4: return keyboard.f4Key;
            case KeyCode.F5: return keyboard.f5Key;
            case KeyCode.F6: return keyboard.f6Key;
            case KeyCode.F7: return keyboard.f7Key;
            case KeyCode.F8: return keyboard.f8Key;
            case KeyCode.F9: return keyboard.f9Key;
            case KeyCode.F10: return keyboard.f10Key;
            case KeyCode.F11: return keyboard.f11Key;
            case KeyCode.F12: return keyboard.f12Key;

            default: return null;
        }
    }
}
