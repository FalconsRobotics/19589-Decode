package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Subsystems.SubsystemCollection;

import java.util.function.DoubleSupplier;

public class IntakeSetPowerCommand extends InstantCommand {
    private final SubsystemCollection sys;
    private final double power;

    public IntakeSetPowerCommand(double power) {
        sys = SubsystemCollection.getInstance(null);

        this.power = power;

//        addRequirements(sys.intake);
    }

    public void initialize() {
        sys.intake.setPower(power);
    }
}
