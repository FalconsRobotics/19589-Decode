package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commands.RobotDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

import old_do_not_touch.Commands.Drivebase.DriveRobotCentricCommand;

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
    }
}