package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Controller;

@TeleOp(name = "Color Sensor")
public class ColorSensorTesting extends LinearOpMode {
    CarouselSubsystem carousel;
    public Controller control;

    public void runOpMode() {
        control = new Controller(gamepad1, gamepad2);
        carousel = new CarouselSubsystem(hardwareMap);


        while (opModeInInit()) {

        }

        while (opModeIsActive()) {
            telemetry.addData("Red", carousel.colorSensorRed());
            telemetry.addData("Green", carousel.colorSensorGreen());
            telemetry.addData("Blue", carousel.colorSensorBlue());
            telemetry.update();
        }

    }
}
