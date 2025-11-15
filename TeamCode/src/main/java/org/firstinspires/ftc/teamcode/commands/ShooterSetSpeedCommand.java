package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Command used to set the intake speed.
 */
public class ShooterSetSpeedCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier speed;

    /**
     * Initializes the ShooterSetSpeedCommand.
     * @param suppliedShooter Uses the drivebase from your running OpMode.
     * @param suppliedSpeed Reference to the speed you want the intake to spin to, in rpms.
     */
    public ShooterSetSpeedCommand(ShooterSubsystem suppliedShooter, DoubleSupplier suppliedSpeed) {
        this.shooter = suppliedShooter;
        this.speed = suppliedSpeed;

        addRequirements(this.shooter);
    }

    @Override
    public void execute() {
        this.shooter.setVelocity(speed.getAsDouble());
    }
}
