package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwerveDriveBase extends SubsystemBase {
    private Translation2d leftModule = new Translation2d(-0.1185, (double) 230/3);
    private Translation2d rightModule = new Translation2d(0.1185, (double) 230/3);
    private Translation2d backModule = new Translation2d(0, (double) 460/3);

    private MotorEx leftDriveMotor;
    private MotorEx rightDriveMotor;
    private MotorEx backDriveMotor;
    private MotorEx pivotMotor;
    private PIDController angleController;
    private GyroEx imu;

    private static final double kP_Angle = 0.005;
    private static final double kI_Angle = 0.001;
    private static final double kD_Angle = 0.0001;

    private SwerveModuleState frontLeftModuleState;
    private SwerveModuleState frontRightModuleState;
    private SwerveModuleState backModuleState;

    private SwerveDriveKinematics swerveKinematics;
    private SwerveDriveOdometry swerveOdometry;

    private Rotation2d targetRotation;

    public SwerveDriveBase(HardwareMap map, Pose2d starting_position) {
        leftDriveMotor = new MotorEx(map, "LeftDrive");
        rightDriveMotor = new MotorEx(map, "RightDrive");
        leftDriveMotor = new MotorEx(map, "BackDrive");
        pivotMotor = new MotorEx(map, "PivotMotor");

        leftDriveMotor.setRunMode(Motor.RunMode.RawPower);
        rightDriveMotor.setRunMode(Motor.RunMode.RawPower);
        backDriveMotor.setRunMode(Motor.RunMode.RawPower);

        angleController = new PIDController(kP_Angle, kI_Angle, kD_Angle);
        pivotMotor.setRunMode(Motor.RunMode.PositionControl);
        angleController.setIntegrationBounds(-Math.PI, Math.PI);

        imu.init();
        imu.reset();

        swerveKinematics = new SwerveDriveKinematics(leftModule, rightModule, backModule);
        swerveOdometry = new SwerveDriveOdometry(
                swerveKinematics, imu.getRotation2d(), starting_position
        );
    }
}
