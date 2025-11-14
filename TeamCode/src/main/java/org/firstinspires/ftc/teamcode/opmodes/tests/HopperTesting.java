package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;

import java.math.BigInteger;

@TeleOp(name = "Hopper Testing", group = "Test") @Config
public class HopperTesting extends LinearOpMode {
    public HopperSubsystem hopper;
    public GamepadEx gp;
    public static double TRIGGER_SCALER = 0.05;

    public void runOpMode() {
        hopper = new HopperSubsystem(hardwareMap);
        gp = new GamepadEx(gamepad1);

        while (opModeInInit()) {

        }

        while (opModeIsActive()) {

            // In your OpMode:
            NormalizedRGBA topColors = hopper.getTopColor();
            NormalizedRGBA bottomColors = hopper.getBottomColor();



            if (gp.wasJustPressed(GamepadKeys.Button.A)) {
                hopper.toPosition(0);
            }
            else {
                hopper.setServoPower(
                        0.0
                                + (gp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * TRIGGER_SCALER)
                                - (gp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * TRIGGER_SCALER)
                );
            }


            gp.readButtons();

            telemetry.addData("Encoder Position", hopper.getServoPosition());
            telemetry.addData("Servo Power", hopper.getServoPower());
            telemetry.addLine("----------------------------------------");
            telemetry.addData(
                    "Top Sensor (R/G/B)",
                    "R: %.2f G: %.2f B: %.2f",
                    topColors.red,
                    topColors.green,
                    topColors.blue
            );
            telemetry.addData(
                    "Bottom Sensor (R/G/B)",
                    "R: %.2f G: %.2f B: %.2f",
                    bottomColors.red,
                    bottomColors.green,
                    bottomColors.blue
            );
            telemetry.addData("Top Distance", hopper.getTopDistance());
            telemetry.addData("Bottom Distance", hopper.getBottomDistance());
            telemetry.update();
        }
    }
}
