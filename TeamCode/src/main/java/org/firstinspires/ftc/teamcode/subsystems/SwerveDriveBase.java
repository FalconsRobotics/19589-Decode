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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SwerveDriveBase extends SubsystemBase {
    private Translation2d leftModule = new Translation2d(-0.1185, (double) 230/3);
    private Translation2d rightModule = new Translation2d(0.1185, (double) 230/3);
    private Translation2d backModule = new Translation2d(0, (double) 460/3);

    private MotorEx leftDriveMotor;
    private MotorEx rightDriveMotor;
    private MotorEx backDriveMotor;
    private MotorEx pivotMotor;
    private PIDController angleController;
    private GoBildaPinpointDriver pinpoint;

    private static final double kP_Angle = 0.005;
    private static final double kI_Angle = 0.001;
    private static final double kD_Angle = 0.0001;

    private Rotation2d targetRotation;

    public SwerveDriveBase(HardwareMap map, Pose2d starting_position) {
        // Initialize all of our motors.
        leftDriveMotor = new MotorEx(map, "LeftDrive");
        rightDriveMotor = new MotorEx(map, "RightDrive");
        leftDriveMotor = new MotorEx(map, "BackDrive");
        pivotMotor = new MotorEx(map, "PivotMotor");

        // Set run modes to directly set power to the driving wheels.
        leftDriveMotor.setRunMode(Motor.RunMode.RawPower);
        rightDriveMotor.setRunMode(Motor.RunMode.RawPower);
        backDriveMotor.setRunMode(Motor.RunMode.RawPower);

        // Setup the pivot motor for the right modes and reset the encoder to zero.
        pivotMotor.resetEncoder();
        pivotMotor.setRunMode(Motor.RunMode.PositionControl);

        // Set up the IMU, initializing it, setting offsets from the computer,
        // set the resolution to match the 4-bar odometry pods, set the direction
        // to forward, reset the position and heading, and recalibrate the IMU.
        pinpoint.initialize();
        pinpoint.setOffsets(0.0, 0.0, DistanceUnit.MM);
        // TODO: Place odometry pods and computer, locate offsets
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();
    }

    private void setDrivePower(double x, double y, double a, double angle) {
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);
        double rotationX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotationY = x * Math.sin(heading) + y * Math.cos(heading);
        double power = Math.sqrt(Math.pow(rotationX, 2) + Math.pow(rotationY, 2));

        double wheelAngle = Math.atan2(rotationX, rotationY);
        double actualAngle = ;
    }
}
