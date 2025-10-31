package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.CarouselPosition;
import org.firstinspires.ftc.teamcode.Constants.ColorConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ControllerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselSubsystem;

@TeleOp(name = "Hopper Test - Nico")
public class TestHopper extends LinearOpMode {
    public ControllerSubsystem control;
    public CarouselSubsystem hopper;
    public boolean checkColorRan = false;

    public void runOpMode() {
        control = new ControllerSubsystem(gamepad1, gamepad2);
        hopper = new CarouselSubsystem(hardwareMap);

        ControllerSubsystem.Toggle autoMode = new ControllerSubsystem.Toggle(false);
        ControllerSubsystem.Toggle nextBall = new ControllerSubsystem.Toggle(false);
        ColorConstants.RGB rgb = new ColorConstants.RGB(0,0,0);
        ColorConstants.BallColor ballColor = ColorConstants.BallColor.NULL;

        int ballsInCounter = 0;

        while(opModeInInit()){
            control.readControllers();
            if (control.base.wasJustPressed(GamepadKeys.Button.A)) {
                hopper.toPos(0.2);
            }
            telemetry.addData("Pos", hopper.getPosDouble());
            telemetry.update();
        }

        while (opModeIsActive()) {
            control.readControllers();
            hopper.updateLEDColors();
            rgb.setRGB(hopper.colorSensorRed(), hopper.colorSensorGreen(), hopper.colorSensorBlue());
            double distance = hopper.getDistance();

            if (control.base.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                hopper.setCounter(hopper.getCounter() + 1);
                hopper.updateBallColors();
            }
            else if (control.base.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                hopper.setCounter(hopper.getCounter() - 1);
            }
            else if (control.base.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                hopper.setCounter(11);
            }
            else if (control.base.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                hopper.setCounter(0);
            }

            hopper.toPos(CarouselPosition.servoPosition(hopper.getCounter()));

            if (distance <= CarouselPosition.distanceMax) {
                if (!checkColorRan) {
                    hopper.updateBallColors();
                    checkColorRan = true;
                }
                ballColor = ColorConstants.detectColor(rgb);
            }
            else {
                checkColorRan = false;
                ballColor = ColorConstants.BallColor.NULL;
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