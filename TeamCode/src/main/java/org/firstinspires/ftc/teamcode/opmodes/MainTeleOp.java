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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.FieldDriveLockCommand;
import org.firstinspires.ftc.teamcode.commands.HopperCycleCCWCommand;
import org.firstinspires.ftc.teamcode.commands.HopperCycleCWCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSetPowerCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterSetSpeedCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp(name = "TeleOp - Open Warfare")
public class MainTeleOp extends CommandOpMode {
    // Objects to store our subsystems.
    public DrivebaseSubsystem drive;
    public IntakeSubsystem intake;
    public HopperSubsystem hopper;
    public ShooterSubsystem shooter;
    public VisionSubsystem vision;

    // Objects to store our Gamepads, using the SolversLib GamepadEx to take advantage
    // of its conveniences for tracking button presses.
    private GamepadEx Gamepad1, Gamepad2;

    // If Red, this is true. If Blue, this is false.
    private boolean isRedAlliance = false;

    // If starting from far, this is false. If starting from close, this is true;
    private boolean isStartingClose = false;

    // Stores whether we are trying to lock to a heading or not.
    private boolean goalLock = false;

    // Stores the last angle we want to lock the heading to.
    private double lastHeadingLock = 0.0;

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
        //endregion

        //region Field-Centric Driving Code
        /// ==================================================
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

        //region Hopper Control Code
        /// ==================================================

        Gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileActiveOnce(
                new HopperCycleCWCommand(hopper)
        );

        Gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileActiveOnce(
                new HopperCycleCCWCommand(hopper)
        );

        /// ==================================================
        //endregion

        //region Extake/Shooter Control Code
        /// ==================================================

        shooter.setDefaultCommand(new ShooterSetSpeedCommand(shooter, () -> 4500));

        /// ==================================================
        //endregion
    }
}