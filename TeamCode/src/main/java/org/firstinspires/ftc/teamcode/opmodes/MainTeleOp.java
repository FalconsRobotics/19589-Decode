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
    public DrivebaseSubsystem drive;

    GamepadEx Gamepad1, Gamepad2;

    @Override
    public void initialize() {
        drive = new DrivebaseSubsystem(hardwareMap);

        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        register(drive);

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
                return 1.0; // Negative is left
            } else if (Gamepad1.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                return -1.0; // Positive is right
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