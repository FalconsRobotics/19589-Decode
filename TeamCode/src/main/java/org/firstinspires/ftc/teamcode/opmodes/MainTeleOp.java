package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.commands.FaceAllianceGoalCommand;
import org.firstinspires.ftc.teamcode.commands.FieldDriveCommand;
import org.firstinspires.ftc.teamcode.commands.FieldDriveLockCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSetPowerCommand;
import org.firstinspires.ftc.teamcode.commands.PeriodicFunctionCommand;
import org.firstinspires.ftc.teamcode.commands.RobotDriveCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterSetSpeedCommand;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@TeleOp(name = "TeleOp - Open Warfare")
public class MainTeleOp extends CommandOpMode {
    // Objects to store our subsystems.
    public DrivebaseSubsystem drive;
    public IntakeSubsystem intake;
    public HopperSubsystem hopper;
    public ShooterSubsystem shooter;
    public VisionSubsystem vision;

    public double angleDifference;

    // Objects to store our Gamepads, using the SolversLib GamepadEx to take advantage
    // of its conveniences for tracking button presses.
    GamepadEx Gamepad1, Gamepad2;

    // If Red, this is true. If Blue, this is false.
    public boolean isRedAlliance = false;

    // If starting from far, this is false. If starting from close, this is true;
    public boolean isStartingClose = false;

    @Override
    public void initialize_loop() {
        // Set the current alliance by pressing DPAD_LEFT or DPAD_RIGHT.
        // Set starting close by pressing X, far by pressing B.
        telemetry.update();
        telemetry.addLine("RED: DPAD_LEFT, BLUE: DPAD_RIGHT");
        telemetry.addLine("CLOSE: X, FAR: B");
        telemetry.addData("Is Red", isRedAlliance);
        telemetry.addData("Is Close", isStartingClose);

        if (Gamepad1.isDown(GamepadKeys.Button.DPAD_LEFT)) {
            isRedAlliance = true;
        } else if (Gamepad1.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            isRedAlliance = false;
        }

        if (Gamepad1.isDown(GamepadKeys.Button.X)) {
            isStartingClose = true;
        } else if (Gamepad1.isDown(GamepadKeys.Button.B)) {
            isStartingClose = false;
        }
    }

    @Override
    public void initialize() {
        // Initialize our subsystems by passing in the HardwareMap, so they can each initialize
        // their own motors, servos, etc.
        drive = new DrivebaseSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, this.drive);

        vision.ll.pipelineSwitch(1);

        // Initialize the FTC Dashboard Telemetry instance so we can print and graph data on the web console.
        Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();

        // Initialize our gamepads by passing in the existing built-in FTC gamepad objects.
        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        waitForStart();

        // Tell SolversLib that this OpMode needs the drivebase and intake to function.
        register(drive, intake, shooter, vision);

        // Schedule any commands that need to be repeated for the entire OpMode.
        schedule(new PeriodicFunctionCommand(this.drive, this.intake, this.shooter, this.vision));

        //region Telemetry
        /// ==================================================

        // Schedule the telemetry updating inside a new command that has no exit condition, meaning
        // it will run throughout the entire OpMode.
        schedule(new CommandBase() {
            @Override
            public void execute() {
                telemetry.update();
                telemetry.addData("Odo X", drive.odo.getPosX(DistanceUnit.INCH));
                telemetry.addData("Odo Y", drive.odo.getPosY(DistanceUnit.INCH));
                telemetry.addData("Odo A", drive.odo.getHeading(AngleUnit.DEGREES));

                telemetry.addData("Shooter Speed", shooter.getVelocity());
                telemetry.addData("R Angle", Math.toDegrees(Math.atan2(-Gamepad1.getRightY(), -Gamepad1.getRightX())));
                dashboard.addData("Shooter Speed", shooter.getVelocity());

                dashboard.update();

            }
        });

        /// ==================================================
        //endregion

        //region Robot-Centric Driving Code
        /// ==================================================

        // When no other Command needs the drivebase, we want it to automatically drive in
        // robot-centric mode. We pass in our drivebase so that the RobotDriveCommand knows
        // what drivebase to use, and we pass in DoubleSuppliers for direct access to our
        // Gamepad joystick values.
//        drive.setDefaultCommand(new RobotDriveCommand(drive, Gamepad1::getLeftY, Gamepad1::getLeftX, Gamepad1::getRightX));

        /// ==================================================
        //endregion

        //region Field-Centric Driving Code
        /// ==================================================

        // SolversLib wants us to use DoubleSuppliers when passing in input values into our
        // commands, so that's what we're doing here. This is a fancy way of saying:
        // - if DPAD_UP is pressed, return 1.0 for the y input (positive y, or forward)
        // - if DPAD_DOWN is pressed, return -1.0 for the y input (negative y, or backward)
        // - if none are pressed, return 0.0 for the y input (no y/forward/backward movement)
        DoubleSupplier fieldForwardSupplier = () -> {
            if (Gamepad1.getButton(GamepadKeys.Button.DPAD_UP)) {
                return 1.0;
            } else if (Gamepad1.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                return -1.0;
            }
            return 0.0;
        };

        // Same as before. This is a fancy way of saying:
        // - if DPAD_LEFT is pressed, return 1.0 for the x input (positive y, or forward)
        // - if DPAD_RIGHT is pressed, return -1.0 for the x input (negative y, or backward)
        // - if none are pressed, return 0.0 for the x input (no x/strafe movement)
        // Why left is positive and right is negative, I don't know. Robots are funny. TODO: Fix later.
        DoubleSupplier fieldStrafeSupplier = () -> {
            if (Gamepad1.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                return 1.0;
            } else if (Gamepad1.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                return -1.0;
            }
            return 0.0;
        };

        // Tell SolversLib to keep track of any of the DPAD buttons. If any of them (up, down,
        // left, right) are pressed at any moment, the trigger will activate. You'll see this in
        // use lower.
        Trigger dpadTrigger = new Trigger(() ->
                Gamepad1.getButton(GamepadKeys.Button.DPAD_UP) ||
                Gamepad1.getButton(GamepadKeys.Button.DPAD_DOWN) ||
                Gamepad1.getButton(GamepadKeys.Button.DPAD_LEFT) ||
                Gamepad1.getButton(GamepadKeys.Button.DPAD_RIGHT)
        );

        // Whenever the dpadTrigger is activated (with any of the DPAD keys being pressed above),
        // take over the drivebase and drive in field-centric mode, with the fieldStrafeSupplier
        // we created earlier controlling the lateral movement of the robot, and the fieldForwardSupplier
        // controlling the forward/backward movement of the robot. The final parameter would ordinarily
        // control rotation of the robot, but we don't want any rotation, so we pass a DoubleSupplier
        // lambda to it with a value of 0.0 so the robot doesn't rotate.
        dpadTrigger.whileActiveContinuous(
                new FieldDriveCommand(
                        drive,
                        fieldStrafeSupplier, // Use our D-pad logic
                        fieldForwardSupplier,  // Use our D-pad logic
                        () -> 0.0
                )
        );

        Gamepad1.getGamepadButton(GamepadKeys.Button.Y).whileHeld(
                new FieldDriveLockCommand(drive, Gamepad1::getLeftX, Gamepad1::getLeftY, drive.normalizeTo180Deg(Math.toDegrees(Math.atan2(72 - drive.odo.getPosX(DistanceUnit.INCH), -72 - drive.odo.getPosY(DistanceUnit.INCH)))))
        );

        Gamepad1.getGamepadButton(GamepadKeys.Button.A).whileHeld(
                new FieldDriveLockCommand(drive, Gamepad1::getLeftX, Gamepad1::getLeftY, drive.normalizeTo180Deg(Math.toDegrees(Math.atan2(72 - drive.odo.getPosX(DistanceUnit.INCH), 72 - drive.odo.getPosY(DistanceUnit.INCH)))))
        );

        new Trigger(() -> Gamepad1.getRightX() >= 0.5).whileActiveContinuous(
                new FieldDriveLockCommand(drive, Gamepad1::getLeftX, Gamepad1::getLeftY, drive.normalizeTo180Deg(Math.toDegrees(Math.atan2(Gamepad1.getRightX(), -Gamepad1.getRightY()))))
        );

        //endregion

        //region Intake Control Code

        // The Intake should, by default, run at 1.0 speed to intake balls in.
        intake.setDefaultCommand(new IntakeSetPowerCommand(intake, () -> 1.0));

        // Whenever the Utility driver
        new Trigger(() -> Gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5)

                .whileActiveContinuous(
                new IntakeSetPowerCommand(intake, () -> -1.0)
        );

        //endregion

        //region Extake/Shooter Control Code
        /// ==================================================

        shooter.setDefaultCommand(new ShooterSetSpeedCommand(shooter, () -> 4500));

        /// ==================================================
        //endregion
    }
}