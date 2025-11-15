package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;

@TeleOp(name = "Sensor Testing", group = "test")
public class SensorTesting extends LinearOpMode {
    public HopperSubsystem hopper;
    public Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();
    public void runOpMode() {
        hopper = new HopperSubsystem(hardwareMap);

        int cycleCounter = 0;

        int cRed = 0; int cGreen = 0; int cBlue = 0;

        int aRed; int aGreen; int aBlue;

        int lowRed = 0; int lowGreen = 0; int lowBlue = 0;
        int highRed = 0; int highGreen = 0; int highBlue = 0;


        while (opModeInInit()) {
            hopper.runToMagnetZero();
        }

        while (opModeIsActive()) {

            int red = hopper.getBottomColor().red;
            int green = hopper.getBottomColor().green;
            int blue = hopper.getBottomColor().blue;

            if (cycleCounter == 0) {
                lowRed = red; highRed = red;
                lowGreen = green; highGreen = green;
                lowBlue = blue; highBlue = blue;
            }

            cycleCounter += 1;

            cRed += red;
            cGreen += green;
            cBlue += blue;

            aRed = cRed / cycleCounter;
            aGreen = cGreen / cycleCounter;
            aBlue = cBlue / cycleCounter;

            if (red < lowRed) {lowRed = red;} if (green < lowGreen) {lowGreen = green;} if (blue < lowBlue) {lowBlue = blue;}
            if (red > highRed) {highRed = red;} if (green > highGreen) {highGreen = green;} if (blue > highBlue) {highBlue = blue;}

            telemetry.setMsTransmissionInterval(200);

            telemetry.addData("Sensor Distance", hopper.getBottomDistance());
            telemetry.addLine("----------------------------------------");
            telemetry.addData("Counter", cycleCounter);
            telemetry.addLine("----------------------------------------");
            telemetry.addData("Current Red", red);
            telemetry.addData("Current Red", green);
            telemetry.addData("Current Red", blue);
            telemetry.addLine("----------------------------------------");
            telemetry.addData("Avg Red", aRed);
            telemetry.addData("Avg Green", aGreen);
            telemetry.addData("Avg Blue", aBlue);
            telemetry.addLine("----------------------------------------");
            telemetry.addData("Red Thresholds", "Low Red: %d | High Red: %d", lowRed, highRed);
            telemetry.addData("Green Thresholds", "Low Red: %d | High Red: %d", lowGreen, highGreen);
            telemetry.addData("Blue Thresholds", "Low Red: %d | High Red: %d", lowBlue, highBlue);

            if (cycleCounter <= 1000) {
                telemetry.update();
            }
        }
    }
}
