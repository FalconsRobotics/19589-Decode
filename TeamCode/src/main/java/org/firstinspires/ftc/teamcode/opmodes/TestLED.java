package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ColorDriver;
import org.firstinspires.ftc.teamcode.controller.Controllers;

@TeleOp (name = "LED Testing (Nico)", group = "Test")
public class TestLED extends LinearOpMode {
    ColorDriver led = new ColorDriver(hardwareMap, true);
    Controllers control = new Controllers(gamepad1, gamepad2);
    double colorTracker = ColorDriver.Color.MAX;

    public void runOpMode() {
        while (opModeInInit()) {}

        while (opModeIsActive()) {
            if (control.baseControl.isDown(GamepadKeys.Button.A)) {
                led.setLedColor(ColorDriver.LED.CHAINED, ColorDriver.Color.SAGE);
            }
            else if (control.baseControl.isDown(GamepadKeys.Button.B)) {
                led.setLedColor(ColorDriver.LED.CHAINED, ColorDriver.Color.RED);
            }
            else if (control.baseControl.isDown(GamepadKeys.Button.X)) {
                led.setLedColor(ColorDriver.LED.CHAINED, ColorDriver.Color.BLUE);
            }
            else if (control.baseControl.isDown(GamepadKeys.Button.Y)) {
                led.setLedColor(ColorDriver.LED.CHAINED, ColorDriver.Color.YELLOW);
            }
            else {
                // If Buttons arent being used -v
                if (control.baseControl.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 && control.baseControl.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 0) {
                    led.setLedColor(ColorDriver.LED.CHAINED, ColorDriver.Color.WHITE);
                }
                else {
                    led.setLedColor(ColorDriver.LED.CHAINED, colorTracker);

                    if (colorTracker > ColorDriver.Color.MAX) { colorTracker = ColorDriver.Color.MAX;}
                    else if (colorTracker < ColorDriver.Color.MIN) { colorTracker = ColorDriver.Color.MIN;}

                    if (control.baseControl.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0 &&
                            control.baseControl.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 &&
                            colorTracker < ColorDriver.Color.MAX) {

                        colorTracker += 0.001;
                    }
                    else if (control.baseControl.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0 &&
                            control.baseControl.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 0  &&
                            colorTracker > ColorDriver.Color.MIN) {

                        colorTracker -= 0.001;
                    }
                }


            }
        }
    }
}
