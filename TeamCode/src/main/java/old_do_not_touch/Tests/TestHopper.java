package old_do_not_touch.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import old_do_not_touch.Subsystems.ControllerSubsystem;
import old_do_not_touch.Subsystems.CarouselSubsystem;

@Disabled
@TeleOp(name = "Hopper Test - Nico", group = "a")
public class TestHopper extends LinearOpMode {
    public ControllerSubsystem control;
    public CarouselSubsystem hopper;
    public boolean checkColorRan = false;

    public void runOpMode() {
        control = new ControllerSubsystem(gamepad1, gamepad2);
        hopper = new CarouselSubsystem(hardwareMap);

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

        }
    }
}