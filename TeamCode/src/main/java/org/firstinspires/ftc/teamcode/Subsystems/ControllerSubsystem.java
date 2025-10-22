package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ControllerSubsystem {
    public GamepadEx base, util;

    public ControllerSubsystem(Gamepad control1, Gamepad control2) {
        this.base = new GamepadEx(control1);
        this.util = new GamepadEx(control2);
    }

    public void initController(Gamepad control1, Gamepad control2) {
        this.base = new GamepadEx(control1);
        this.util = new GamepadEx(control2);
    }

    public void initBaseControl(Gamepad control) {this.base = new GamepadEx(control);}

    public void initUtilControl(Gamepad control) {this.util = new GamepadEx(control);}

    public static class Toggle {
        private boolean toggled;

        public Toggle(boolean state) {
            this.toggled = state;
        }
        public boolean isTrue() {
            return this.toggled;
        }
        public void toggle() {
            if (this.toggled == true) {
                this.setFalse();
            }
            else if (this.toggled == false) {
                this.setTrue();
            }
        }
        public void setTrue() {
            this.toggled = true;
        }
        public void setFalse() {
            this.toggled = false;
        }
    }

}
