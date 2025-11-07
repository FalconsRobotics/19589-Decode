package old_do_not_touch.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import old_do_not_touch.Constants.CarouselPosition;
import old_do_not_touch.Subsystems.CarouselSubsystem;
import old_do_not_touch.Subsystems.ControllerSubsystem;

@Disabled
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
