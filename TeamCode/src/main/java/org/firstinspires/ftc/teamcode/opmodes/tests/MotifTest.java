package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

@TeleOp(name = "Simple Motif Test", group = "Test")
public class MotifTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Create subsystems
        DrivebaseSubsystem drive = new DrivebaseSubsystem(hardwareMap);
        VisionSubsystem vision = new VisionSubsystem(hardwareMap, drive);

        telemetry.addLine("Ready to detect motif...");
        telemetry.update();

        waitForStart();

        // Run until we get a nonzero motif
        while (opModeIsActive()) {
            vision.periodic(); // update Limelight & odometry data
            vision.findMotif();

            int motif = vision.getMotif();

            telemetry.addData("Motif Value", motif);
            telemetry.update();

            // Stop once a valid motif is found
            if (motif != 0) {
                telemetry.addLine("Motif detected!");
                telemetry.update();
                break;
            }

            sleep(50); // small delay to prevent spam
        }

        // Keep displaying the final result
        while (opModeIsActive()) {
            telemetry.addData("Final Motif", vision.getMotif());
            telemetry.update();
            sleep(100);
        }
    }
}
