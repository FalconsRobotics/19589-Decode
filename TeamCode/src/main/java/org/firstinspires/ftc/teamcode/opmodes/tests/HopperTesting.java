package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;

@TeleOp(name = "Hopper Testing", group = "Test") @Config
public class HopperTesting extends LinearOpMode {
    public HopperSubsystem hopper;

    public void runOpMode() {
        hopper = new HopperSubsystem(hardwareMap);

        hopper.findHopperHomePosition();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.rightBumperWasPressed()) hopper.setTargetPosition(1);
            if (gamepad1.leftBumperWasPressed()) hopper.setTargetPosition(-1);

            hopper.rotateHopperOnePosition(1);
            hopper.readHopperPosition();

            telemetry.addData("Encoder Ticks", hopper.hopperEncoderTicks);
            telemetry.addData("Servo Power", hopper.hopperServoPower);
            telemetry.addData("Hopper Position", hopper.hopperPosition);
            telemetry.addData("P", hopper.hopperPTerm);
            telemetry.addData("D", hopper.hopperDTerm);
            telemetry.addData("Error", hopper.hopperEncoderTicksError);

            telemetry.update();
        }
    }

}
