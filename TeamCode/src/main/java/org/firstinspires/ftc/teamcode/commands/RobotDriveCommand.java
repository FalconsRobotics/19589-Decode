package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

import java.util.function.DoubleSupplier;

public class RobotDriveCommand extends CommandBase {
    // An object used to reference our existing Drivebase, pulled from the constructor,
    // which gets the subsystem from our OpMode.
    private final DrivebaseSubsystem drive;
    // DoubleSuppliers that track the gamepad inputs, and one that handles speed multipliers
    // that pull from the gamepad trigger.
    private final DoubleSupplier forwardPower;
    private final DoubleSupplier strafePower;
    private final DoubleSupplier turnPower;
    private final DoubleSupplier speedMultiplier;

    public RobotDriveCommand(DrivebaseSubsystem suppliedDrive, DoubleSupplier suppliedForward, DoubleSupplier suppliedStrafe, DoubleSupplier suppliedTurn, DoubleSupplier suppliedSpeed) {
        // Set the internal members to our passed-in values, so that this command uses the inputs
        // and subsystems from our OpMode.
        this.drive = suppliedDrive;
        this.forwardPower = suppliedForward;
        this.strafePower = suppliedStrafe;
        this.turnPower = suppliedTurn;
        this.speedMultiplier = suppliedSpeed;

        // Tell SolversLib that we need to use the DrivebaseSubsystem in this command.
        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        // Call the DrivebaseSubsystem drive function using our supplied movement values.
        this.drive.drive(-strafePower.getAsDouble() * this.speedMultiplier.getAsDouble(), -forwardPower.getAsDouble() * this.speedMultiplier.getAsDouble(), -turnPower.getAsDouble() * this.speedMultiplier.getAsDouble());
    }
}