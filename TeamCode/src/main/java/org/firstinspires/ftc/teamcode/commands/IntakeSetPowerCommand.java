package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Command used to set the intake power.
 */
public class IntakeSetPowerCommand extends CommandBase {
    // An object used to reference our existing Intake, pulled from the constructor,
    // which gets the subsystem from our OpMode.
    private final IntakeSubsystem intake;

    // A DoubleSupplier used to directly access the trigger
    private final DoubleSupplier power;

    /**
     * Initializes the IntakeSetPowerCommand.
     * @param suppliedIntake Uses the intake from your running OpMode.
     * @param suppliedPower Reference to the intake power you want the intake to spin to.
     */
    public IntakeSetPowerCommand(IntakeSubsystem suppliedIntake, DoubleSupplier suppliedPower) {
        // Set the internal members to our passed-in values, so that this command uses the inputs
        // and subsystems from our OpMode.
        this.intake = suppliedIntake;
        this.power = suppliedPower;

        // Tell SolversLib that we need to use the DrivebaseSubsystem in this command.
        addRequirements(this.intake);
    }

    @Override
    public void execute() {
        // Call the IntakeSubsystem setIntakePower function using our supplied power.
        intake.setIntakePower(this.power.getAsDouble());
    }
}