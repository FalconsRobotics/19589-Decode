package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Compare Odo, MT1, and MT2", group = "Test")
public class CompareOdoMT1MT2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        DrivebaseSubsystem drive = new DrivebaseSubsystem(hardwareMap);
        VisionSubsystem vision = new VisionSubsystem(hardwareMap, drive);

        vision.ll.pipelineSwitch(1);

        telemetry.addLine("Ready to compare Odo, MT1, and MT2 poses...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update Limelight and odometry
            vision.periodic();

            // Get poses
            Pose2D odoPose = drive.odo.getPosition();
            Pose2D mt1Pose = vision.llPoseMT1();
            Pose2D mt2Pose = vision.llPose();

            // --- Odometry ---
            if (odoPose != null) {
                telemetry.addData("Odo (mm, deg)",
                        "X: %.1f  Y: %.1f  H: %.1f",
                        odoPose.getX(DistanceUnit.MM),
                        odoPose.getY(DistanceUnit.MM),
                        odoPose.getHeading(AngleUnit.DEGREES));
            } else telemetry.addLine("Odo: (no data)");

            // --- MT1 Pose ---
            if (mt1Pose != null) {
                telemetry.addData("MT1 Pose (mm, deg)",
                        "X: %.1f  Y: %.1f  H: %.1f",
                        mt1Pose.getX(DistanceUnit.MM),
                        mt1Pose.getY(DistanceUnit.MM),
                        mt1Pose.getHeading(AngleUnit.DEGREES));
            } else telemetry.addLine("MT1: (no data)");

            // --- MT2 Pose ---
            if (mt2Pose != null) {
                telemetry.addData("MT2 Pose (mm, deg)",
                        "X: %.1f  Y: %.1f  H: %.1f",
                        mt2Pose.getX(DistanceUnit.MM),
                        mt2Pose.getY(DistanceUnit.MM),
                        mt2Pose.getHeading(AngleUnit.DEGREES));
            } else telemetry.addLine("MT2: (no data)");

            telemetry.update();
            sleep(50);
        }
    }
}
