package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.color.ColorDriver;
import org.firstinspires.ftc.teamcode.color.Gradient;
import org.firstinspires.ftc.teamcode.controller.Controllers;
import org.firstinspires.ftc.teamcode.color.Color;
import org.firstinspires.ftc.teamcode.color.LED;

@TeleOp (name = "LED Testing 10/08/25", group = "Test")
public class TestLED extends LinearOpMode {
    public ColorDriver led;
    Controllers control;
    Gradient.CyclicGradient gradientCycle = new Gradient.CyclicGradient(Color.RED);
    double colorTracker = Color.MAX;

    public void runOpMode() {
        led = new ColorDriver(hardwareMap);
        control = new Controllers(gamepad1,gamepad2);

        while (opModeInInit()) {}

        while (opModeIsActive()) {
            if (control.baseControl.isDown(GamepadKeys.Button.A)) {
                led.setLedColor(LED.CHAINED, Color.SAGE);
            }
            else if (control.baseControl.isDown(GamepadKeys.Button.B)) {
                led.setLedColor(LED.CHAINED, Color.RED);
            }
            else if (control.baseControl.isDown(GamepadKeys.Button.X)) {
                led.setLedColor(LED.CHAINED, Color.BLUE);
            }
            else if (control.baseControl.isDown(GamepadKeys.Button.Y)) {
                led.setLedColor(LED.CHAINED, Color.YELLOW);
            }
            else if (control.baseControl.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                led.setLedColor(LED.CHAINED, gradientCycle.getNextPosition(Color.RED, Color.VIOLET));
            }
            else {
                led.setLedColor(LED.CHAINED, colorTracker);

                /// Makes the color displayed stay in boundary
                if (colorTracker > Color.MAX) { colorTracker = Color.MAX;}
                else if (colorTracker < Color.MIN) { colorTracker = Color.MIN;}

                /// Positive Direction -^
                if (
                        control.baseControl.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0 &&
                        control.baseControl.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0 &&
                                colorTracker < Color.MAX) { colorTracker += 0.001;}
                /// Negative Direction -v
                else if (
                        control.baseControl.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0 &&
                        control.baseControl.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 0  &&
                                colorTracker > Color.MIN) { colorTracker -= 0.001;}
            }

            telemetry.addData("Displayed Color (double)", led.getLedColor(LED.CHAINED));
            telemetry.update();
        }
    }
}
