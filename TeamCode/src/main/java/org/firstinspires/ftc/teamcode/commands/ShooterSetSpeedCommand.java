package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterSetSpeedCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier speed;

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
