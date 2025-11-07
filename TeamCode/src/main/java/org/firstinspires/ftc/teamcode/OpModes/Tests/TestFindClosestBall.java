package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Find Closest Ball - Teigan", group = "Test")
public class TestFindClosestBall extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialize once ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(3); // Ensure this is your SnapScript pipeline index
        limelight.start();

        telemetry.addLine("Limelight initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            limelight.start();
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double[] llpython = result.getPythonOutput();

                if (llpython != null && llpython.length >= 4) {
                    int numBalls = llpython.length / 4;

                    for (int i = 0; i < numBalls; i++) {
                        double color = llpython[i * 4];
                        double cx = llpython[i * 4 + 1];
                        double cy = llpython[i * 4 + 2];
                        double area = llpython[i * 4 + 3];

                        String colorName = (color == 1.0) ? "Purple" :
                                (color == 2.0) ? "Green" : "Unknown";

                        telemetry.addData("Ball %d", i + 1);
                        telemetry.addData(" - Color", colorName);
                        telemetry.addData(" - X", cx);
                        telemetry.addData(" - Y", cy);
                        telemetry.addData(" - Area", area);
                    }
                } else {
                    telemetry.addLine("No Python output detected yet.");
                }

                telemetry.addData("Pipeline", result.getPipelineIndex());
            } else {
                telemetry.addLine("No valid Limelight result yet.");
            }

            assert result != null;
            telemetry.addData("Python Output: ", result.getPythonOutput());
            telemetry.addData("Pipeline Name", result.getPipelineType());

            telemetry.update();

            sleep(50);
        }
    }
}
