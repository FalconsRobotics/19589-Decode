package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

public class Controller {
    public GamepadEx base, util;

    public Controller(Gamepad control1, Gamepad control2) {
        this.base = new GamepadEx(control1);
        this.util = new GamepadEx(control2);
    }

    public void initController(Gamepad control1, Gamepad control2) {
        this.base = new GamepadEx(control1);
        this.util = new GamepadEx(control2);
    }

    public void readControllers() {
        this.base.readButtons();
        this.util.readButtons();
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
            if (this.toggled) {
                this.setFalse();
            }
            else {
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
