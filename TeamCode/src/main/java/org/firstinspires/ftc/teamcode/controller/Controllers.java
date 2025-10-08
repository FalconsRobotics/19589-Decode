package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Controllers {
    public GamepadEx baseControl, utilControl;
    public static class baseButtons {

    }
    public static class utilButtons {
        public static GamepadKeys.Button shoot = GamepadKeys.Button.RIGHT_BUMPER;
    }

    public Controllers(Gamepad control1, Gamepad control2) {
        this.baseControl = new GamepadEx(control1);
        this.utilControl = new GamepadEx(control2);
    }
    public Controllers(Gamepad control, int number) {
        if (number == 1) {
            this.baseControl = new GamepadEx(control);
        }
        else if (number == 2) {
            this.utilControl = new GamepadEx(control);
        }
        else {
            return;
        }
    }

    public void initController(Gamepad control1, Gamepad control2) {
        this.baseControl = new GamepadEx(control1);
        this.utilControl = new GamepadEx(control2);
    }

    public void initBaseControl(Gamepad control) {this.baseControl = new GamepadEx(control);}

    public void initUtilControl(Gamepad control) {this.utilControl = new GamepadEx(control);}



}
