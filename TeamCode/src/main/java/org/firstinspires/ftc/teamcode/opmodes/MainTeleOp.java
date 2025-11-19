package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.FieldDriveLockCommand;
import org.firstinspires.ftc.teamcode.commands.HopperCycleCCWCommand;
import org.firstinspires.ftc.teamcode.commands.HopperCycleCWCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSetPowerCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterSetSpeedCommand;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(name = "TeleOp - Open Warfare")
public class MainTeleOp extends CommandOpMode {
    // Objects to store our subsystems.
    public DrivebaseSubsystem drive;
    public IntakeSubsystem intake;
    public HopperSubsystem hopper;
    public ShooterSubsystem shooter;
    public VisionSubsystem vision;

    public Telemetry dashboard;

    // Objects to store our Gamepads, using the SolversLib GamepadEx to take advantage
    // of its conveniences for tracking button presses.
    private GamepadEx Gamepad1, Gamepad2;

    // If Red, this is true. If Blue, this is false.
    private boolean isRedAlliance = true;

    // Stores the last angle we want to lock the heading to.
    private double lastHeadingLock = 0.0;

    @Override
    public void initialize_loop() {
        // Set the current alliance by pressing DPAD_LEFT or DPAD_RIGHT.
        // Set starting close by pressing X, far by pressing B.
        telemetry.update();
        telemetry.addLine("RED: DPAD_LEFT, BLUE: DPAD_RIGHT");
//        telemetry.addLine("CLOSE: X, FAR: B");
        telemetry.addData("Is Red", isRedAlliance);
//        telemetry.addData("Is Close", isStartingClose);

        if (Gamepad1.isDown(GamepadKeys.Button.DPAD_LEFT)) {
            isRedAlliance = true;
        } else if (Gamepad1.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            isRedAlliance = false;
        }
    }

    // TODO: Convert Utility functions to the Utility Gamepad.
    @Override
    public void initialize() {
        // Initialize our subsystems by passing in the HardwareMap, so they can each initialize
        // their own motors, servos, etc.
        drive = new DrivebaseSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        hopper = new HopperSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, this.drive);

        // Initialize the FTC Dashboard Telemetry instance so we can print and graph data on the web console.
        dashboard = FtcDashboard.getInstance().getTelemetry();

        // Initialize our gamepads by passing in the existing built-in FTC gamepad objects.
        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        // Tell SolversLib that this OpMode needs the drivebase, intake, hopper, shooter, and vision to function.
        register(drive, intake, hopper, shooter, vision);

        //region Driving Code
        /// ==================================================

        // Unless the drivebase is going to be used by another command, the default thing it should do is just drive field-centrically.
        drive.setDefaultCommand(
            new FieldDriveLockCommand(
                    drive,
//                    Gamepad1::getLeftX,
//                    Gamepad1::getLeftY,
                    () -> isRedAlliance ? Gamepad1.getLeftY() : -Gamepad1.getLeftY(),
                    () -> isRedAlliance ? -Gamepad1.getLeftX() : Gamepad1.getLeftX(),
                    () -> lastHeadingLock
            )
        );

        // The robot needs to indefinitely track whether the joystick magnitude is greater than 0.5, and set a new heading angle if so.
        // It checks if it's greater than 0.5 because returning the joystick to (0, 0) sets the angle to 0, and we need to prevent accidental touches.
        // We do this by making a trigger that repeatedly checks whenever the joystick magnitude is greater than 0.5
        new Trigger(() -> Math.hypot(Gamepad1.getRightX(), Gamepad1.getRightY()) >= 0.5).whileActiveContinuous(
            new InstantCommand(() -> {
                if (isRedAlliance) {
                    lastHeadingLock = Math.toDegrees(Math.atan2(Gamepad1.getRightY(), -Gamepad1.getRightX()));
                } else {
                    lastHeadingLock = Math.toDegrees(Math.atan2(-Gamepad1.getRightY(), Gamepad1.getRightX()));
                }
            })
        );

        // When pressing X, we want to aim towards the respective goal when in the close position.
        Gamepad1.getGamepadButton(GamepadKeys.Button.X).whileHeld(
                new FieldDriveLockCommand(
                        drive,
                        () -> isRedAlliance ? Gamepad1.getLeftY() : -Gamepad1.getLeftY(),
                        () -> isRedAlliance ? -Gamepad1.getLeftX() : Gamepad1.getLeftX(),
                        () -> isRedAlliance ? -45 : 45
                )
        );

        // When pressing B, we want to aim towards the respective goal when in the far position.
        Gamepad1.getGamepadButton(GamepadKeys.Button.B).whileHeld(
                new FieldDriveLockCommand(
                        drive,
                        () -> isRedAlliance ? Gamepad1.getLeftY() : -Gamepad1.getLeftY(),
                        () -> isRedAlliance ? -Gamepad1.getLeftX() : Gamepad1.getLeftX(),
                        () -> isRedAlliance ? -27.5 : 27.5
                )
        );

        //endregion

        //region Intake Control Code
        /// ==================================================

        // The Intake should, by default, run at 1.0 speed to intake balls in.
        intake.setDefaultCommand(new IntakeSetPowerCommand(intake, () -> 1.0));

        // Whenever the Utility driver presses the right trigger more than halfway, the intake should reverse.
        new Trigger(() -> Gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5)
            .whileActiveContinuous(
                new IntakeSetPowerCommand(intake, () -> -1.0)
            );

        /// ==================================================
        //endregion

        //region Hopper Control Code
        /// ==================================================

        // When Utility presses the Left Bumper, we want to rotate clockwise, which shoots a ball.
        // But, we only want to do this whenever
//        Gamepad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileActiveOnce(
//            // Conditional command: If the condition (either shooter is in the correct speed range OR the "override" A button is pressed)
//            // is true, only then do we want to shoot. Otherwise, don't shoot, and just rumble the controller to signal to Utility that
//            // you're not in the correct velocity range.
//            new ConditionalCommand(
//                    new HopperCycleCWCommand(hopper),
//                    new InstantCommand(() -> gamepad1.rumbleBlips(3)),
//                    () -> shooter.isInTargetSpeed() || Gamepad1.getButton(GamepadKeys.Button.A)
//            )
//        );
//
//        // When Utility presses the Right Bumper, we want to rotate counterclockwise, which intakes a ball.
//        Gamepad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileActiveOnce(
//                new HopperCycleCCWCommand(hopper)
//        );

        /// ==================================================
        //endregion

        //region Extake/Shooter Control Code
        /// ==================================================

        shooter.setDefaultCommand(new ShooterSetSpeedCommand(shooter, () -> 4500));

        /// ==================================================
        //endregion

        //region Miscellaneous Code
        /// ==================================================

        // We need an emergency heading reset, just in case!
        Gamepad2.getGamepadButton(GamepadKeys.Button.OPTIONS).whileActiveOnce(new CommandBase() {
            @Override
            public void execute() {
                 drive.resetHeadingPID();
            }
        });

        /// ==================================================
        //endregion
    }

    @Override
    public void run() {
        telemetry.update();
        telemetry.addData("Odo X", drive.odo.getPosX(DistanceUnit.INCH));
        telemetry.addData("Odo Y", drive.odo.getPosY(DistanceUnit.INCH));
        telemetry.addData("Odo A", drive.odo.getHeading(AngleUnit.DEGREES));

        telemetry.addData("Last Heading Target", lastHeadingLock);
        telemetry.addData("Stick Hypot", Math.hypot(Gamepad1.getRightX(), Gamepad1.getRightY()));

        telemetry.addData("Shooter Target Speed", shooter.isInTargetSpeed());
        telemetry.addData("R Angle", Math.toDegrees(Math.atan2(-Gamepad1.getRightY(), -Gamepad1.getRightX())));

        dashboard.update();

        super.run();
    }
}