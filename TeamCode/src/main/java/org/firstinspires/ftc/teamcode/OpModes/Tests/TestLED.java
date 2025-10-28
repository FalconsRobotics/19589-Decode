package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Color.Color;
import org.firstinspires.ftc.teamcode.Subsystems.Color.LEDSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Color.LED;
import org.firstinspires.ftc.teamcode.Subsystems.ControllerSubsystem;


//@TeleOp (name = "LED Testing 10/08/25", group = "Test")
@Disabled
public class TestLED extends LinearOpMode {
    public LEDSubsystem led;
    ControllerSubsystem control;
    double colorTracker = Color.MIN;
    ControllerSubsystem.Toggle allMode;
    Color.Pattern.CyclicGradient gradientCycle = new Color.Pattern.CyclicGradient(Color.RED);
    Color.Pattern.StraightGradient gradientStraight = new Color.Pattern.StraightGradient(Color.RED);
    Color.Pattern.Flash flashPattern = new Color.Pattern.Flash(Color.WHITE);
    Color.Pattern.SmoothTo smoothTo = new Color.Pattern.SmoothTo(colorTracker);

    public void runOpMode() {
        led = new LEDSubsystem(hardwareMap);
        control = new ControllerSubsystem(gamepad1,gamepad2);
        allMode = new ControllerSubsystem.Toggle(true);
        String debug = "null";

        while (opModeInInit()) {
            if (control.base.isDown(GamepadKeys.Button.A)) {
                led.setLedColor(LED.ALL, smoothTo.getNextPosition(Color.SAGE));
            } else if (control.base.isDown(GamepadKeys.Button.B)) {
                led.setLedColor(LED.ALL, smoothTo.getNextPosition(Color.RED));
            } else if (control.base.isDown(GamepadKeys.Button.X)) {
                led.setLedColor(LED.ALL, smoothTo.getNextPosition(Color.AZURE));
            } else if (control.base.isDown(GamepadKeys.Button.Y)) {
                led.setLedColor(LED.ALL, smoothTo.getNextPosition(Color.YELLOW));
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
                        control.base.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0 &&
                                control.base.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 &&
                                colorTracker < Color.MAX) {
                    colorTracker += 0.001;
                }
                /// Negative Direction -v
                else if (
                        control.base.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0 &&
                                control.base.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 0 &&
                                colorTracker > Color.MIN) {
                    colorTracker -= 0.001;
                }
            }
        }

        while (opModeIsActive()) {
            if (control.base.isDown(GamepadKeys.Button.A)) {
                led.setLedColor(LED.ALL, Color.SAGE);
            } else if (control.base.isDown(GamepadKeys.Button.B)) {
                led.setLedColor(LED.ALL, Color.RED);
            } else if (control.base.isDown(GamepadKeys.Button.X)) {
                led.setLedColor(LED.ALL, Color.BLUE);
            } else if (control.base.isDown(GamepadKeys.Button.Y)) {
                led.setLedColor(LED.ALL, Color.YELLOW);
            } else if (control.base.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                led.setLedColor(LED.ALL, gradientCycle.getNextPosition(Color.RED, Color.VIOLET));
            } else if (control.base.isDown(GamepadKeys.Button.DPAD_UP)) {
                led.setLedColor(LED.ALL, Color.WHITE);
            } else if (control.base.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
                led.setLedColor(LED.ALL, gradientStraight.getNextPosition(Color.RED, Color.YELLOW));
            } else if (control.base.isDown(GamepadKeys.Button.DPAD_LEFT)) {
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
                        control.base.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0 &&
                                control.base.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 &&
                                colorTracker < Color.MAX) {
                    colorTracker += 0.001;
                }
                /// Negative Direction -v
                else if (
                        control.base.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0 &&
                                control.base.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 0 &&
                                colorTracker > Color.MIN) {
                    colorTracker -= 0.001;
                }
            }

            telemetry.addData("Displayed Color (double)", led.getLedColor(LED.CHAINED));
            telemetry.update();
        }
    }
}
