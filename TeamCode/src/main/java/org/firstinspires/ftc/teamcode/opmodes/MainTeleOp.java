package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.FieldDriveCommand;
import org.firstinspires.ftc.teamcode.commands.RobotDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

import java.util.function.DoubleSupplier;

@TeleOp(name = "TeleOp - Open Warfare")
public class MainTeleOp extends CommandOpMode {
    // Objects to store our subsystems.
    public DrivebaseSubsystem drive;

    // Objects to store our Gamepads, using the SolversLib GamepadEx to take advantage
    // of its conveniences for tracking button presses.
    GamepadEx Gamepad1, Gamepad2;

    @Override
    public void initialize() {
        // Initialize our subsystems by passing in the HardwareMap, so they can each initialize
        // their own motors, servos, etc.
        drive = new DrivebaseSubsystem(hardwareMap);

        // Intialize our gamepads by passing in the existing built-in FTC gamepad objects.
        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        // Tell SolversLib that this OpMode needs the DrivebaseSubsystem to function.
        register(drive);

        // When no other Command needs the drivebase, we want it to automatically drive in
        // robot-centric mode.
        drive.setDefaultCommand(new RobotDriveCommand(drive, Gamepad1::getLeftY, Gamepad1::getLeftX, Gamepad1::getRightX));

        DoubleSupplier fieldForwardSupplier = () -> {
            if (Gamepad1.getButton(GamepadKeys.Button.DPAD_UP)) {
                return 1.0;
            } else if (Gamepad1.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                return -1.0;
            }
            return 0.0;
        };

        DoubleSupplier fieldStrafeSupplier = () -> {
            if (Gamepad1.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                return 1.0;
            } else if (Gamepad1.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                return -1.0;
            }
            return 0.0;
        };

        Trigger dpadTrigger = new Trigger(() ->
                Gamepad1.getButton(GamepadKeys.Button.DPAD_UP) ||
                Gamepad1.getButton(GamepadKeys.Button.DPAD_DOWN) ||
                Gamepad1.getButton(GamepadKeys.Button.DPAD_LEFT) ||
                Gamepad1.getButton(GamepadKeys.Button.DPAD_RIGHT)
        );

        dpadTrigger.whileActiveContinuous(
                new FieldDriveCommand(
                        drive,
                        fieldForwardSupplier, // Use our D-pad logic
                        fieldStrafeSupplier,  // Use our D-pad logic
                        () -> 0.0
                )
        );
    }

    @Override
    public void run() {
        super.run();
    }
}