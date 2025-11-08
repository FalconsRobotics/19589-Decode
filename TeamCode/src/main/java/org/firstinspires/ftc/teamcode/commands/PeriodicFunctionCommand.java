package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class PeriodicFunctionCommand extends CommandBase {
    private final DrivebaseSubsystem driveInstance;
    private final IntakeSubsystem intakeInstance;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;

    public PeriodicFunctionCommand(DrivebaseSubsystem suppliedDrive, IntakeSubsystem suppliedIntake, ShooterSubsystem suppliedShooter, VisionSubsystem suppliedVision) {
        this.driveInstance = suppliedDrive;
        this.intakeInstance = suppliedIntake;
        this.shooter = suppliedShooter;
        this.vision = suppliedVision;
    }

    @Override
    public void execute() {
        driveInstance.periodic();
        vision.periodic();
    }
}
