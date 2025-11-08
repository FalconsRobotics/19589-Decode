package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

import java.util.function.DoubleSupplier;

public class FieldDriveLockCommand extends CommandBase {
    // An object used to reference our existing Drivebase, pulled from the constructor,
    // which gets the subsystem from our OpMode.
    private final DrivebaseSubsystem drive;
    // DoubleSuppliers that track the gamepad inputs.
    private final DoubleSupplier forwardPower;
    private final DoubleSupplier strafePower;
    private final DoubleSupplier angleTurn;

    public FieldDriveLockCommand(DrivebaseSubsystem suppliedDrive, DoubleSupplier suppliedStrafe, DoubleSupplier suppliedForward, DoubleSupplier suppliedTurn) {
        // Set the internal members to our passed-in values, so that this command uses the inputs
        // and subsystems from our OpMode.
        this.drive = suppliedDrive;
        this.forwardPower = suppliedForward;
        this.strafePower = suppliedStrafe;
        this.angleTurn = suppliedTurn;

        // Tell SolversLib that we need to use the DrivebaseSubsystem in this command.
        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        // Call the DrivebaseSubsystem drive function using our supplied movement values.
        this.drive.driveFieldCentric(strafePower.getAsDouble(), -forwardPower.getAsDouble(), angleTurn.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        if (this.drive.odo.getHeading(AngleUnit.DEGREES) >= angleTurn.getAsDouble() - 5.0 &&
                this.drive.odo.getHeading(AngleUnit.DEGREES) <= angleTurn.getAsDouble() + 5.0) return true;

        return false;
    }
}