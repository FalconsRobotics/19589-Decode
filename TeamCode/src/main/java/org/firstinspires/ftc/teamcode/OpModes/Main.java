package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import java.util.List;

@TeleOp(name = "TeleOp - Open Warfare", group = "Main")
public class Main extends OpMode {
    // Right now, we're not going to use SubsystemCollection, the singleton
    // that holds all of our subsystem classes, until we start using SolversLib.
    // SubsystemCollection sys;

    // Objects for each of our subsystems
    MecanumDriveBase drivebase;
    IntakeElevatorSubsystem intake;
    CarouselSubsystem hopper;
    ShooterSubsystem shooter;

    // Here, we decided to use SolversLib's GamepadEx controller for its
    // "whenJustPressed()" and "isDown()" functionality to maintain simplicity.
    GamepadEx Gamepad1;
    GamepadEx Gamepad2;

    @Override
    public void init() {
        // Set up the various subsystems.
        // TODO: When adding commands, add in SubsystemsCollection
        drivebase = new MecanumDriveBase(hardwareMap);
        intake = new IntakeElevatorSubsystem(hardwareMap);
        hopper = new CarouselSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        // Initialize the Gamepads for use from SolversLib.
        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        // Enable bulk cached reading to speed up I2C read times.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void start() {
        // As soon as the match starts (not when we init, when the match actually starts
        // and we're allowed to move), we want the shooter to get up to speed.
//        shooter.setPower(ShooterConstants.FIRING_SPEED);
    }

    @Override
    public void loop() {
        // Call the periodic functions from the SubsystemCollection
        // class. Functions that need to be run every loop.
        drivebase.periodic();
        hopper.periodic();
        shooter.periodic();

        // SolversLib requires that we read the gamepad every loop.
        Gamepad1.readButtons();
        Gamepad2.readButtons();

        // Subsystem control
        shooter.setPower(ShooterConstants.FIRING_SPEED);
        intake.setPower(1.0);

        // Gather joystick values from LX, LY (inverted, so up is forward y),
        // and RX (for turning) on Gamepad1.
        double cX = Gamepad1.getLeftX();
        double cY = -Gamepad1.getLeftY();
        double cRX = Gamepad1.getRightX();
        double cRY = -Gamepad1.getRightY();

        double turnAngle = Math.atan2(cRX, cRY);
        double speedMultiplier = 1 - Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        drivebase.DriveFieldCentricWithLock(cX * speedMultiplier, cY * speedMultiplier, turnAngle);

        // Controls for the hopper. LB cycles it left, RB cycles it right a single time.
        // D-Pad down returns it back to zero / the center;
        if (Gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            hopper.setCounter(hopper.getCounter() - 1);
        } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            hopper.setCounter(hopper.getCounter() + 1);
        } else if (Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            hopper.setCounter(0);
        }

        // If the right trigger is pressed (at least halfway, range doesn't really matter),
        // make sure the shooter is at the optimal speed or check if the override (left bumper
        // in this case) is pressed, then shoot all three balls.
        if (Gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5) {
            if (shooter.isInPowerBand() || Gamepad2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                hopper.setCounter(hopper.getCounter() - 3);
            }
        }

        telemetry.addLine("------------------------------");
        telemetry.addData("Counter", hopper.getCounter());
        telemetry.addData("Servo Position", hopper.getPosDouble());
        telemetry.addLine("------------------------------");
        telemetry.addData("Shooter Velocity", shooter.getVelocity());
        telemetry.update();
    }
}