package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;

@TeleOp(name = "Limelight Testing - Teigan", group = "Test")
public class LimelightTesting extends LinearOpMode {
    LimelightSubsystem ll;


    public void runOpMode() {
        ll = new LimelightSubsystem(hardwareMap);

        while (opModeInInit()) {}

        while (opModeIsActive()) {
           // telemetry.addData("Motif (int)", ll.getMotif());
            telemetry.addData("Pipeline: ", ll.getPipeline());
            telemetry.addData("Angle to Blue", ll.angleToGoalBlue());
            telemetry.addData("Angle to Red", ll.angleToGoalRed());
            telemetry.addData("Closest ball: ", ll.findClosestBall());
            telemetry.update();
            ll.refreshLL();

        }

    }
}
