package org.firstinspires.ftc.teamcode.Commands.Drivebase;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.SubsystemCollection;

import java.util.function.DoubleSupplier;

public class DriveRobotCentricCommand extends CommandBase {
    private final SubsystemCollection sys;
    private final double x, y, a;

    public DriveRobotCentricCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier a) {
        sys = SubsystemCollection.getInstance(null);

        this.x = x.getAsDouble();
        this.y = y.getAsDouble();
        this.a = a.getAsDouble();

//        addRequirements(sys.drivebase);
    }

    public void execute() {
        sys.drivebase.Drive(this.x, this.y, this.a, -1);
    }

    public void end() {
        sys.drivebase.Drive(0, 0, 0, -1);
    }

    public boolean isFinished() {
        return false;
    }
}
