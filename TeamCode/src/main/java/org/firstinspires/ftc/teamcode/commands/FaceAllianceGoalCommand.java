package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

//Command allows you to drive freely in the x and y direction
//while the robot automatically turns and locks in on the goal
@Config
public class FaceAllianceGoalCommand extends CommandBase {
    private final DrivebaseSubsystem drive;
    private final VisionSubsystem vision;
    private final DoubleSupplier x;     // strafe input (robot or field based; pass what your subsystem expects)
    private final DoubleSupplier y;     // forward input
    private final BooleanSupplier isRed;

    private final double kP;
    private final double maxTurn;
    private final double tolDeg;
    private final double deadbandDeg;

    public FaceAllianceGoalCommand(
            DrivebaseSubsystem drive, VisionSubsystem vision,
            DoubleSupplier x, DoubleSupplier y, BooleanSupplier isRed,
            double kP, double maxTurn, double tolDeg, double deadbandDeg) {
        this.drive = drive;
        this.vision = vision;
        this.x = x;
        this.y = y;
        this.isRed = isRed;
        this.kP = kP;
        this.maxTurn = maxTurn;
        this.tolDeg = tolDeg;
        this.deadbandDeg = deadbandDeg;
        // Tell SolversLib that we need to use the DrivebaseSubsystem in this command.
        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        // Driver translation still live:
        double strafe = x.getAsDouble();
        double forward = y.getAsDouble();

        // Vision angle (deg). Positive = target left of robot.
        double angleDeg = vision.angleToAllianceGoal(isRed.getAsBoolean());

        double rotCmd = 0.0;
        if (!Double.isNaN(angleDeg)) {
            if (Math.abs(angleDeg) > deadbandDeg) {
                rotCmd = Math.max(-maxTurn, Math.min(maxTurn, kP * angleDeg));
            }
        } else {
            // Vision lost -> optional fallback: hold current heading using your turnPID
            // or just pass 0 so driver keeps control.
            rotCmd = 0.0;
        }

        // Use your preferred frame here:
//        this.drive.driveFieldCentricHeadingLock(strafe, forward, rotCmd);
    }

    @Override
    public boolean isFinished() {
        // Assisted hold is usually continuous; let the trigger release cancel it.
        // If you want an auto-finish, return true when within tolerance WITH a stable vision lock:
        // double e = drive.angleToAllianceGoal(isRed.getAsBoolean());
        // return !Double.isNaN(e) && Math.abs(e) < tolDeg;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // optional: nothing, or stop rotation smoothing, etc.
    }
}
