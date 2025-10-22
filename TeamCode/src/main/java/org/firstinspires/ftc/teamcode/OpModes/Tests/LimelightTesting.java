package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Limelight;

@TeleOp(name = "Limelight Testing - Teigan", group = "Test")
public class LimelightTesting extends LinearOpMode {
    Limelight ll;

    public void runOpMode() {
        ll = new Limelight(hardwareMap);

        while (opModeInInit()) {}

        while (opModeIsActive()) {
            telemetry.addData("Motif (int)", ll.getMotif());
            telemetry.addData("Angle to Blue", ll.angleToGoalBlue());
            telemetry.addData("Angle to Red", ll.angleToGoalRed());
            telemetry.update();

        }

    }
}
