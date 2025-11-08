package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Simple Motif + Pose + Odo Test", group = "Test")
public class SimpleMotifPoseOdoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialize subsystems ---
        DrivebaseSubsystem drive = new DrivebaseSubsystem(hardwareMap);
        VisionSubsystem vision = new VisionSubsystem(hardwareMap, drive);

        telemetry.addLine("Ready to detect motif...");
        telemetry.update();

        waitForStart();

        // --- Main Loop ---
        while (opModeIsActive()) {
            // Update subsystems
            vision.periodic();

            // Run detection
            vision.findMotif();
            int motif = vision.getMotif();

            // Get LL Pose and Odo Pose
            Pose2D llPose = vision.llPose();
            Pose2D odoPose = drive.odo.getPosition();

            telemetry.addData("Motif Value", motif);

            // --- Limelight Pose ---
            if (llPose != null) {
                telemetry.addData("Limelight Pose (mm, deg)",
                        "X: %.1f  Y: %.1f  Heading: %.1f",
                        llPose.getX(DistanceUnit.MM),
                        llPose.getY(DistanceUnit.MM),
                        llPose.getHeading(AngleUnit.DEGREES));
            } else {
                telemetry.addLine("Limelight Pose: (no data yet)");
            }

            // --- Odometry Pose ---
            if (odoPose != null) {
                telemetry.addData("Odo Pose (mm, deg)",
                        "X: %.1f  Y: %.1f  Heading: %.1f",
                        odoPose.getX(DistanceUnit.MM),
                        odoPose.getY(DistanceUnit.MM),
                        odoPose.getHeading(AngleUnit.DEGREES));
            } else {
                telemetry.addLine("Odo Pose: (no data yet)");
            }

            telemetry.update();

            // Stop once a motif is detected
            if (motif != 0) {
                telemetry.addLine("Motif detected!");
                telemetry.update();
                break;
            }

            sleep(50);
        }

        // --- Keep showing final data ---
        while (opModeIsActive()) {
            Pose2D llPose = vision.llPose();
            Pose2D odoPose = drive.odo.getPosition();

            telemetry.addData("Final Motif", vision.getMotif());

            if (llPose != null) {
                telemetry.addData("Final LL Pose (mm, deg)",
                        "X: %.1f  Y: %.1f  Heading: %.1f",
                        llPose.getX(DistanceUnit.MM),
                        llPose.getY(DistanceUnit.MM),
                        llPose.getHeading(AngleUnit.DEGREES));
            }

            if (odoPose != null) {
                telemetry.addData("Final Odo Pose (mm, deg)",
                        "X: %.1f  Y: %.1f  Heading: %.1f",
                        odoPose.getX(DistanceUnit.MM),
                        odoPose.getY(DistanceUnit.MM),
                        odoPose.getHeading(AngleUnit.DEGREES));
            }

            telemetry.update();
            sleep(100);
        }
    }
}
