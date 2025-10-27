package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.CarouselPosition;
import org.firstinspires.ftc.teamcode.Subsystems.Color.Color;
import org.firstinspires.ftc.teamcode.Subsystems.Controller;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselSubsystem;

@TeleOp(name = "Hopper Test - Nico")
public class TestHopper extends LinearOpMode {
    public Controller control;
    public CarouselSubsystem hopper;

    public void runOpMode() {
        control = new Controller(gamepad1, gamepad2);
        hopper = new CarouselSubsystem(hardwareMap);

        Controller.Toggle autoMode = new Controller.Toggle(false);
        Controller.Toggle nextBall = new Controller.Toggle(false);
        Color.RGB rgb = new Color.RGB(0,0,0);
        Color.BallColor ballColor = Color.BallColor.NULL;

        int ballsInCounter = 0;

        while(opModeInInit()){
            control.readControllers();
            if (control.base.wasJustPressed(GamepadKeys.Button.A)) {
                hopper.toPos(0.2);
            }
        }

        while (opModeIsActive()) {
            control.readControllers();
            hopper.updateColors();
            rgb.setRGB(hopper.colorSensorRed(), hopper.colorSensorGreen(), hopper.colorSensorBlue());
            double distance = hopper.getDistance();

            if (control.base.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                hopper.setCounter(hopper.getCounter() + 1);
                hopper.updateBalls();
            } else if (control.base.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                hopper.setCounter(hopper.getCounter() - 1);
            } else if (control.base.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                hopper.setCounter(11);
            } else if (control.base.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                hopper.setCounter(0);
            }

            hopper.toPos(CarouselPosition.servoPosition(hopper.getCounter()));

            if (distance <= CarouselPosition.distanceMax) {
                ballColor = Color.detectColor(rgb);
            }
            else {
                ballColor = Color.BallColor.NULL;
            }

            telemetry.addData("AutoMode", autoMode.isTrue());
            telemetry.addLine("----------");
            telemetry.addData("Pos (Servo)", hopper.getPosDouble());
            telemetry.addData("Carousel Counter", hopper.getCounter());
            telemetry.addLine("----------");
            telemetry.addData("Position Equation", CarouselPosition.servoPosition(hopper.getCounter()));
            telemetry.addLine("----------");
            telemetry.addData("Distance (MM", distance);
            telemetry.addLine("----------");
            telemetry.addData("Detected Color", ballColor);
            telemetry.addData("R", rgb.getR());
            telemetry.addData("G", rgb.getG());
            telemetry.addData("B", rgb.getB());
            telemetry.addLine("----------");
            telemetry.addData("Ball 1 Color", hopper.ball1.color);
            telemetry.addData("Ball 2 Color", hopper.ball2.color);
            telemetry.addData("Ball 3 Color", hopper.ball3.color);
            telemetry.update();

        }
    }
}