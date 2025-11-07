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

        //region Robot-Centric Driving Code
        /// ==================================================

        // When no other Command needs the drivebase, we want it to automatically drive in
        // robot-centric mode. We pass in our drivebase so that the RobotDriveCommand knows
        // what drivebase to use, and we pass in DoubleSuppliers for direct access to our
        // Gamepad joystick values.
        drive.setDefaultCommand(new RobotDriveCommand(drive, Gamepad1::getLeftY, Gamepad1::getLeftX, Gamepad1::getRightX));

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

        /// ==================================================
        //endregion

        //region Intake Control
        //endregion
    }

    // Might not be necessary, as the OpMode already runs as-is.
    @Override
    public void run() {
        super.run();
    }
}