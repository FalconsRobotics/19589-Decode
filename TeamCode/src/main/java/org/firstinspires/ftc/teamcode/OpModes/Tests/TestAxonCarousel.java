package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Color.Color;
import org.firstinspires.ftc.teamcode.Subsystems.Controller;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.Constants.CarouselPosition;

@TeleOp(name = "Carousel Test - Nico")
public class TestAxonCarousel extends LinearOpMode {
    public Controller control;
    public CarouselSubsystem carousel;

    public void runOpMode() {
        control = new Controller(gamepad1, gamepad2);
        carousel = new CarouselSubsystem(hardwareMap);

        Controller.Toggle autoMode = new Controller.Toggle(false);
        Controller.Toggle nextBall = new Controller.Toggle(false);
        Color.RGB rgb = new Color.RGB(0,0,0);
        Color.BallColor ballColor = Color.BallColor.NULL;

        int ballsInCounter = 0;

        while(opModeInInit()){

        }

        while (opModeIsActive()) {
            control.readControllers();
            carousel.updateColors();
            rgb.setRGB(carousel.colorSensorRed(), carousel.colorSensorGreen(), carousel.colorSensorBlue());
            double distance = carousel.getDistance();

            //if (control.base.wasJustPressed(GamepadKeys.Button.START)) {autoMode.toggle();}

            if (control.base.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                carousel.setCounter(carousel.getCounter() + 1);
                carousel.updateBalls();
            } else if (control.base.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                carousel.setCounter(carousel.getCounter() - 1);
            } else if (control.base.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                carousel.setCounter(11);
            } else if (control.base.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                carousel.setCounter(0);
            }

            carousel.toPos(CarouselPosition.servoPosition(carousel.getCounter()));

            if (distance <= CarouselPosition.distanceMax) {
                ballColor = Color.detectColor(rgb);
            }
            else {
                ballColor = Color.BallColor.NULL;
            }

            if (nextBall.isTrue()) {

            }

            telemetry.addData("AutoMode", autoMode.isTrue());
            telemetry.addLine("----------");
            telemetry.addData("Pos (Servo)", carousel.getPosDouble());
            telemetry.addData("Carousel Counter", carousel.getCounter());
            telemetry.addLine("----------");
            telemetry.addData("Position Equation", CarouselPosition.servoPosition(carousel.getCounter()));
            telemetry.addLine("----------");
            telemetry.addData("Distance (MM", distance);
            telemetry.addLine("----------");
            telemetry.addData("Detected Color", ballColor);
            telemetry.addData("R", rgb.getR());
            telemetry.addData("G", rgb.getG());
            telemetry.addData("B", rgb.getB());
            telemetry.addLine("----------");
            telemetry.addData("Ball 1 Color", carousel.ball1.color);
            telemetry.addData("Ball 2 Color", carousel.ball2.color);
            telemetry.addData("Ball 3 Color", carousel.ball3.color);
            telemetry.update();

        }
    }
}