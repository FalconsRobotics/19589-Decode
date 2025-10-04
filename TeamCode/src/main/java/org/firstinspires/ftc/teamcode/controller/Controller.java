package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
    public GamepadEx baseControl, utilControl;
    public static class baseButtons {

    }
    public static class utilButtons {
        public static GamepadKeys.Button shoot = GamepadKeys.Button.RIGHT_BUMPER;
    }

    public Controller(Gamepad control1, Gamepad control2) {
        this.baseControl = new GamepadEx(control1);
        this.utilControl = new GamepadEx(control2);

    }

    public void initController(Gamepad control1, Gamepad control2) {
        baseControl = new GamepadEx(control1);
        utilControl = new GamepadEx(control2);
    }



}
