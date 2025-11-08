package org.firstinspires.ftc.teamcode.opmodes.tests;

// Import FTC and hardware classes for Limelight and GoBilda Pinpoint odometry
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * This OpMode demonstrates how to use the Limelight 3A in 3D (AprilTag) mode
 * alongside the GoBilda Pinpoint Odometry to get:
 *  - Your robot’s field position (via MegaTag2 pose)
 *  - Your angle to each visible AprilTag (goal tags)
 */
@TeleOp(name = "Limelight 3d Testing - Teigan", group = "Test")
public class Limelight3dTrackingTest extends LinearOpMode {
    // Stores the 2D pose (x, y, heading)
    Pose2D pose;
    // Stores the latest Limelight result (tag detections, tx, ty, etc.)
    LLResult result;

    @Override
    public void runOpMode() {
        // Get the Limelight from hardware map (name must match Robot Configuration)
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");

        // Select the pipeline to use (0 = default, usually AprilTag detection)
        ll.pipelineSwitch(0);

        // Set how often the Limelight sends data (Hz = updates per second)
        ll.setPollRateHz(67); // 67Hz is close to max for fast updates

        // Start the Limelight connection thread
        ll.start();


        // --- PINPOINT ODOMETRY INITIALIZATION ---

        // Get the GoBilda Pinpoint odometry module
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Initialize odometry hardware
        odo.initialize();

        // Tell the pinpoint which type of encoder pods are used
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set the direction of the encoders (depends on how you mounted them)
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset both the odometry position and the IMU heading to zero
        odo.resetPosAndIMU();

        // Recalibrate the IMU to ensure accurate heading
        odo.recalibrateIMU();


        // --- WAIT FOR DRIVER TO PRESS PLAY ---
        waitForStart();


        // --- MAIN LOOP ---
        // This loop runs repeatedly while the OpMode is active
        while (opModeIsActive()) {

            // Get the latest frame of data from Limelight
            result = ll.getLatestResult();

            // Get the robot’s current heading (degrees) from odometry
            double robotAngle = odo.getHeading(AngleUnit.DEGREES);

            // Update Limelight with your robot’s orientation
            // (needed for MegaTag2 to correctly compute 3D pose)
            ll.updateRobotOrientation(robotAngle);


            // --- GET FIELD POSE FROM MEGATAG2 ---
            if (result != null && result.isValid()) {

                // Get the robot’s 3D pose estimate from MegaTag2 (in meters or mm)
                Pose3D pose3d = result.getBotpose_MT2();

                if (pose3d != null) {
                    // Convert to a 2D pose (x, y, heading)
                    Pose2D botpose_mt2 = new Pose2D(
                            DistanceUnit.MM,
                            pose3d.getPosition().x,  // X position
                            pose3d.getPosition().y,  // Y position
                            AngleUnit.DEGREES,
                            robotAngle               // Use current IMU/odo heading
                    );

                    // Display the 2D position on telemetry, rounded to 3 decimal places
                    telemetry.addData("MT2 Pose (mm, deg)", "(X: %.3f, Y: %.3f, θ: %.3f)",
                            botpose_mt2.getX(DistanceUnit.MM),
                            botpose_mt2.getY(DistanceUnit.MM),
                            botpose_mt2.getHeading(AngleUnit.DEGREES));
                }
            }


            // --- GET ANGLE TO EACH DETECTED APRILTAG ---
            // Only proceed if Limelight has valid detections and fiducial targets exist
            if (result != null && result.isValid()) {
                if (!result.getFiducialResults().isEmpty()) {

                    // Get all AprilTag detections (fiducials)
                    List<LLResultTypes.FiducialResult> fidRes = result.getFiducialResults();

                    // Loop through each tag that was seen
                    for (LLResultTypes.FiducialResult target : fidRes) {

                        // Get the tag ID (e.g., 19 = Red goal, 24 = Blue goal)
                        int tagId = target.getFiducialId();

                        // Get the tag’s position relative to the robot
                        // Prefer RobotSpace if you are updating robot orientation with ll.updateRobotOrientation()
                        Pose3D p = target.getTargetPoseRobotSpace();   // X forward, Y left, Z up
                        if (p == null)
                            p = target.getTargetPoseCameraSpace();     // Fallback: relative to camera

                        // Extract translation values (in same units, usually meters or mm)
                        double x = p.getPosition().x;  // Forward distance to tag
                        double y = p.getPosition().y;  // Left distance to tag

                        // Compute the horizontal angle from robot to tag in degrees
                        double angleDeg = Math.toDegrees(Math.atan2(y, x));

                        // Normalize the angle to [-180, 180] range
                        angleDeg = ((angleDeg + 180) % 360 + 360) % 360 - 180;

                        // Display tag ID and the computed angle to it
                        telemetry.addData("TargetId:", tagId);
                        telemetry.addData("Angle to Goal:", ("θ: %.3f"), angleDeg);
                    }
                }
            }

            // Update telemetry so data appears on Driver Station
            telemetry.update();
        }
    }
}
