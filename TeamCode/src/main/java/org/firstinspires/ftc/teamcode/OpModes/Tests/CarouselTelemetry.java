package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants.CarouselPosition;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ControllerSubsystem;

@TeleOp(name = "Telemetry Carousel")
public class CarouselTelemetry extends LinearOpMode {

    CarouselSubsystem hopper;
    ControllerSubsystem c;

    public void runOpMode() {

        hopper = new CarouselSubsystem(hardwareMap);
        c = new ControllerSubsystem(gamepad1, gamepad2);

        while (opModeInInit()) {
            c.readControllers();
            if (c.base.wasJustPressed(GamepadKeys.Button.A)) {
                hopper.toPos(CarouselPosition.intakeMin);
            }
        }

        while (opModeIsActive()) {
            double pos = hopper.getPosDouble();
            c.readControllers();
            if (c.base.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                hopper.toPos(hopper.getPosDouble() + 0.01);
            }
            else if (c.base.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                hopper.toPos(hopper.getPosDouble() - 0.01);
            }

            telemetry.addData("Carousel Position", pos);
            telemetry.update();
        }

    }
}
