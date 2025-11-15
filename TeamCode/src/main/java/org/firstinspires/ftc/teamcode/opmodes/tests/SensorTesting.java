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

        int aRed = 0; int aGreen = 0; int aBlue = 0;

        int lowRed = 0; int lowGreen = 0; int lowBlue = 0;
        int highRed = 0; int highGreen = 0; int highBlue = 0;


        while (opModeInInit()) {

        }

        while (opModeIsActive()) {

            int red = (int) hopper.getBottomColor().red;
            int green = (int) hopper.getBottomColor().green;
            int blue = (int) hopper.getBottomColor().blue;

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

            telemetry.addData("Sensor Distance", hopper.getBottomDistance());
            telemetry.addLine("----------------------------------------");
            telemetry.addData("Avg Red", aRed);
            telemetry.addData("Avg Green", aGreen);
            telemetry.addData("Avg Blue", aBlue);
            telemetry.addLine("----------------------------------------");
            telemetry.addData("Red Thresholds", "Low Red: %.2f | High Red: %.2f", lowRed, highRed);
            telemetry.addData("Green Thresholds", "Low Red: %.2f | High Red: %.2f", lowGreen, highGreen);
            telemetry.addData("Blue Thresholds", "Low Red: %.2f | High Red: %.2f", lowBlue, highBlue);
            telemetry.update();
            /* ------------------------------------------------------------------ */
            dashboard.addData("Avg Red", aRed);
            dashboard.addData("Avg Green", aGreen);
            dashboard.addData("Avg Blue", aBlue);
            dashboard.addLine("----------------------------------------");
            dashboard.addData("Red Thresholds", "Low Red: %.2f | High Red: %.2f", lowRed, highRed);
            dashboard.addData("Green Thresholds", "Low Red: %.2f | High Red: %.2f", lowGreen, highGreen);
            dashboard.addData("Blue Thresholds", "Low Red: %.2f | High Red: %.2f", lowBlue, highBlue);
            dashboard.update();
        }
    }
}
