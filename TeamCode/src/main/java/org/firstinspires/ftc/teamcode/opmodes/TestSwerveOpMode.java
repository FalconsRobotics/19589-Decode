package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase;

public class TestSwerveOpMode extends OpMode {
    private SwerveDriveBase swerve;
    double x, y, a;

    @Override
    public void init() {
        swerve = new SwerveDriveBase(hardwareMap, new Pose2d(
                new Translation2d(0.0, 0.0),
                new Rotation2d(0)
        ));
    }

    @Override
    public void loop() {
        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;
        a = gamepad1.right_stick_x;
    }
}
