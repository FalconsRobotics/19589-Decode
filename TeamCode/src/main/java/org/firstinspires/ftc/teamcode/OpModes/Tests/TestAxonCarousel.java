package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ControllerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Carousel;
import org.firstinspires.ftc.teamcode.Constants.CarouselConstants;

@TeleOp(name = "Carousel Test - Nico", group = "Test")
public class TestAxonCarousel extends LinearOpMode {
    public ControllerSubsystem control;
    public Carousel carousel;
    public double carouselPower = 0;

    public void runOpMode() {
        control = new ControllerSubsystem(gamepad1, gamepad2);
        carousel = new Carousel(hardwareMap);
        ControllerSubsystem.Toggle nextBall = new ControllerSubsystem.Toggle(false);
        boolean ballDetected = false;
        double errorOutput = 0;
        double newOutput = 0;

        while(opModeInInit()){
            if (control.base.isDown(GamepadKeys.Button.DPAD_LEFT)) {carousel.setInOne(true);}
            if (control.base.isDown(GamepadKeys.Button.DPAD_DOWN)) {carousel.setInTwo(true);}
            if (control.base.isDown(GamepadKeys.Button.DPAD_RIGHT)) {carousel.setInThree(true);}

            carousel.setPower(0);
        }

        while (opModeIsActive()) {
            double distance = carousel.getDistance();
            double position = carousel.getPower();
            double positionInt = carousel.getPosInt();

            if (control.base.isDown(GamepadKeys.Button.X)) {
                errorOutput = carousel.toPosReturn(CarouselConstants.input1);
            }
            else if (control.base.isDown(GamepadKeys.Button.A)) {
                errorOutput = carousel.toPosReturn(CarouselConstants.input2);
            }
            else if (control.base.isDown(GamepadKeys.Button.B)) {
                errorOutput = carousel.toPosReturn(CarouselConstants.input3);
            }
            else if (control.base.isDown(GamepadKeys.Button.DPAD_UP)) {
                errorOutput = carousel.toPosReturn(1);
            }
            else if (control.base.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                errorOutput = carousel.toPosReturn(0);
            }
            else {
                errorOutput = carousel.toPosReturn(0.5);
            }


            telemetry.addData("Pos (V)", carousel.getPos());
            telemetry.addData("New Pos", newOutput);
            telemetry.addData("Power (Raw)", carousel.getPower());
            telemetry.addData("Error", errorOutput);
            telemetry.addLine("----------");
            telemetry.addData("Position (Servo)", position);
            telemetry.addData("nextBall", nextBall.isTrue());
            telemetry.addLine("----------");
            telemetry.addData("Distance (MM", distance);
            telemetry.update();
        }
    }
}
