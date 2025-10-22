package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.color.ColorDriver;
import org.firstinspires.ftc.teamcode.color.Pattern;
import org.firstinspires.ftc.teamcode.controller.Controllers;
import org.firstinspires.ftc.teamcode.color.Color;
import org.firstinspires.ftc.teamcode.color.LED;

@TeleOp (name = "LED Testing 10/08/25", group = "Test")
public class TestLED extends LinearOpMode {
    public ColorDriver led;
    Controllers control;
    Controllers.Toggle allMode;
    Pattern.CyclicGradient gradientCycle = new Pattern.CyclicGradient(Color.RED);
    Pattern.StraightGradient gradientStraight = new Pattern.StraightGradient(Color.RED);
    Pattern.Flash flashPattern = new Pattern.Flash(Color.WHITE);
    double colorTracker = Color.MIN;

    public void runOpMode() {
        led = new ColorDriver(hardwareMap);
        control = new Controllers(gamepad1,gamepad2);
        allMode = new Controllers.Toggle(true);
        String debug = "null";

        while (opModeInInit()) {}

        while (opModeIsActive()) {
            if (control.baseControl.isDown(GamepadKeys.Button.A)) {
                led.setLedColor(LED.ALL, Color.SAGE);
            } else if (control.baseControl.isDown(GamepadKeys.Button.B)) {
                led.setLedColor(LED.ALL, Color.RED);
            } else if (control.baseControl.isDown(GamepadKeys.Button.X)) {
                led.setLedColor(LED.ALL, Color.BLUE);
            } else if (control.baseControl.isDown(GamepadKeys.Button.Y)) {
                led.setLedColor(LED.ALL, Color.YELLOW);
            } else if (control.baseControl.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                led.setLedColor(LED.ALL, gradientCycle.getNextPosition(Color.RED, Color.VIOLET));
            } else if (control.baseControl.isDown(GamepadKeys.Button.DPAD_UP)) {
                led.setLedColor(LED.ALL, Color.WHITE);
            } else if (control.baseControl.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
                led.setLedColor(LED.ALL, gradientStraight.getNextPosition(Color.RED, Color.YELLOW));
            } else if (control.baseControl.isDown(GamepadKeys.Button.DPAD_LEFT)) {
                led.setLedColor(LED.ALL, flashPattern.getFlashPosition(Color.RED, Color.WHITE));
            } else {
                led.setLedColor(LED.ALL, colorTracker);

                /// Makes the color displayed stay in boundary
                if (colorTracker > Color.MAX) {
                    colorTracker = Color.MAX;
                } else if (colorTracker < Color.MIN) {
                    colorTracker = Color.MIN;
                }

                /// Positive Direction -^
                if (
                        control.baseControl.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0 &&
                                control.baseControl.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 &&
                                colorTracker < Color.MAX) {
                    colorTracker += 0.001;
                }
                /// Negative Direction -v
                else if (
                        control.baseControl.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0 &&
                                control.baseControl.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 0 &&
                                colorTracker > Color.MIN) {
                    colorTracker -= 0.001;
                }
            }

            telemetry.addData("Displayed Color (double)", led.getLedColor(LED.CHAINED));
            telemetry.update();
        }
    }
}
