package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

import java.util.function.DoubleSupplier;

public class RobotDriveCommand extends CommandBase {
    private final DrivebaseSubsystem drive;
    private final DoubleSupplier forwardPower;
    private final DoubleSupplier strafePower;
    private final DoubleSupplier turnPower;

    public RobotDriveCommand(DrivebaseSubsystem suppliedDrive, DoubleSupplier forwardSupplied, DoubleSupplier strafeSupplied, DoubleSupplier turnSupplied) {
        this.drive = suppliedDrive;
        this.forwardPower = forwardSupplied;
        this.strafePower = strafeSupplied;
        this.turnPower = turnSupplied;

        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        this.drive.drive(strafePower.getAsDouble(), -forwardPower.getAsDouble(), turnPower.getAsDouble());
    }
}