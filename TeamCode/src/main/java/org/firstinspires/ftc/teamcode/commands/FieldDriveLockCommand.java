package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Command used to drive the robot field-centrically with an additional heading lock instead of a turn power.
 */
public class FieldDriveLockCommand extends CommandBase {
    // An object used to reference our existing Drivebase, pulled from the constructor,
    // which gets the subsystem from our OpMode.
    private final DrivebaseSubsystem drive;
    // DoubleSuppliers that track the gamepad inputs.
    private final DoubleSupplier forwardPower;
    private final DoubleSupplier strafePower;
    private final DoubleSupplier angleTurn;

    /**
     * Initializes the FieldDriveLockCommand.
     * @param suppliedDrive Uses the drivebase from your running OpMode.
     * @param suppliedStrafe Reference to the sideways power you want the robot to move.
     * @param suppliedForward Reference to the forward power you want the robot to move.
     * @param suppliedAngle Reference to the angle you want the robot to lock to.
     */
    public FieldDriveLockCommand(DrivebaseSubsystem suppliedDrive, DoubleSupplier suppliedStrafe, DoubleSupplier suppliedForward, DoubleSupplier suppliedAngle) {
        // Set the internal members to our passed-in values, so that this command uses the inputs
        // and subsystems from our OpMode.
        this.drive = suppliedDrive;
        this.forwardPower = suppliedForward;
        this.strafePower = suppliedStrafe;
        this.angleTurn = suppliedAngle;

        // Tell SolversLib that we need to use the DrivebaseSubsystem in this command.
        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        // Call the DrivebaseSubsystem drive function using our supplied movement values.
        this.drive.driveFieldCentricHeadingLock(strafePower.getAsDouble(), forwardPower.getAsDouble(), angleTurn.getAsDouble());
    }
}