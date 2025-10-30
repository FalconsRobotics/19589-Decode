package org.firstinspires.ftc.teamcode.Commands.Drivebase;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.SubsystemCollection;

public class DriveFieldCentricCommand extends CommandBase {
    private final SubsystemCollection sys;
    private final double x, y, a;

    public DriveFieldCentricCommand(double x, double y, double a) {
        sys = SubsystemCollection.getInstance(null);

        this.x = x;
        this.y = y;
        this.a = a;

//        addRequirements(sys.drivebase);
    }

    public void execute() {
//        sys.drivebase.DriveFieldCentric(this.x, this.y, this.a, -1);
    }
}
