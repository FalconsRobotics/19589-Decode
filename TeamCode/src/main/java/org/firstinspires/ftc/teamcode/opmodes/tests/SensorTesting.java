package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;

@TeleOp(name = "Sensor Testing", group = "test")
public class SensorTesting extends LinearOpMode {
    public HopperSubsystem hopper;

    public void runOpMode() {
        hopper = new HopperSubsystem(hardwareMap);

        while (opModeInInit()) {

        }

        while (opModeIsActive()) {


            telemetry.addData("Sensor Detecting", hopper.getBottomDistance());
            telemetry.update();
        }
    }
}
