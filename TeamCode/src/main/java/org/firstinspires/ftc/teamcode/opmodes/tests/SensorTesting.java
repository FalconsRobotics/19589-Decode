package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;

@TeleOp(name = "Sensor Testing", group = "test")
public class SensorTesting extends LinearOpMode {
    public HopperSubsystem hopper;
    public GamepadEx gp;

    public void runOpMode() {
        hopper = new HopperSubsystem(hardwareMap);
        gp = new GamepadEx(gamepad1);

        while (opModeInInit()) {

        }

        while (opModeIsActive()) {


            telemetry.addData("Sensor Detecting")
            telemetry.update();
        }
    }
}
