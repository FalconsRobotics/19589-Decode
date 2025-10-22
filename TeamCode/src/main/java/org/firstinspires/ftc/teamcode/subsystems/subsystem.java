package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class subsystem {
    public SwerveDriveBase base;
    public Intake intake;

    public void initBase(HardwareMap map, Pose2d startPos) {
        base = new SwerveDriveBase(map, startPos);
    }

    public void initIntake(HardwareMap map) {
        intake = new Intake(map);
    }

    public void initAll(HardwareMap map, Pose2d driveBaseStartPos) {
        initBase(map, driveBaseStartPos);
        initIntake(map);
    }
}
