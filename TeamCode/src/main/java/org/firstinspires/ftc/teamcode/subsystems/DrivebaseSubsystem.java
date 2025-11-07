package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;

public class DrivebaseSubsystem extends SubsystemBase {
    // Motor objects that store the references to the motors on the robot.
    public MotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // This is the MecanumDrive built into SolversLib, and we'll be relying on its
    // built-in functions.
    public MecanumDrive drivebase;

    // Object to store the GoBildaPinpointDriver IMU.
    public GoBildaPinpointDriver odo;

    // Object to hold a PID controller that we will use to rotate the robot.
    public PIDFController turnPID;

    public DrivebaseSubsystem(HardwareMap map) {
        // Initialize the four Mecanum wheel motors.
        frontLeftMotor = map.get(MotorEx.class, "FrontLeft");
        frontLeftMotor = map.get(MotorEx.class, "FrontRight");
        frontLeftMotor = map.get(MotorEx.class, "BackLeft");
        frontLeftMotor = map.get(MotorEx.class, "BackRight");

        // Create the SolversLib MecanumDrive object from those four motors.
        drivebase = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        // Pull the IMU from the HardwareMap and initialize it with the proper settings,
        // using the GoBilda 4-bar odometry pod resolution and forward directions.
        odo = map.get(GoBildaPinpointDriver.class, "odo");
        odo.initialize();
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        // Initialize the PID controller for turning the robot.
        turnPID = new PIDFController(
                DriveConstants.DRIVE_KP,
                DriveConstants.DRIVE_KI,
                DriveConstants.DRIVE_KD,
                DriveConstants.DRIVE_KF
        );
        turnPID.setSetPoint(0);
    }

    /**
     * Calls the SolversLib MecanumDrive driveRobotCentric() function.
     * @param cX The x-value taken from the gamepad, used for strafing.
     * @param cY The y-value taken from the gamepad, used for forward/backward movement.
     * @param cA The a-value taken from the gamepad, used for rotation.
     */
    public void drive(double cX, double cY, double cA) {
        drivebase.driveRobotCentric(cX, cY, cA);
    }

    /**
     * Calls the SolversLib MecanumDrive driveFieldCentric() function. Relies on the
     * built-in GoBildaPinpoint heading rather than passing one in.
     * @param cX The x-value taken from the gamepad, used for strafing.
     * @param cY The y-value taken from the gamepad, used for forward/backward movement.
     * @param cA The a-value taken from the gamepad, used for rotation.
     */
    public void driveFieldCentric(double cX, double cY, double cA) {
        // odo.getHeading() retrieves the current heading from the Pinpoint computer.
        // The SolversLib function requires degrees, so that's what we'll use.
        drivebase.driveFieldCentric(cX, cY, cA, odo.getHeading(AngleUnit.DEGREES));
    }

    /**
     * Calls the SolversLib MecanumDrive driveFieldCentric() function. Relies on the
     * built-in GoBildaPinpoint heading rather than passing one in. Replaces a controller
     * turn angle with a heading that you want the robot to lock to.
     * @param cX The x-value taken from the gamepad, used for strafing.
     * @param cY The y-value taken from the gamepad, used for forward/backward movement.
     * @param lockHeading A custom heading to lock the drivebase to.
     */
    public void driveFieldCentricHeadingLock(double cX, double cY, double lockHeading) {
        double currentHeading = odo.getHeading(AngleUnit.DEGREES);

        double error = AngleUnit.normalizeDegrees(lockHeading - currentHeading);
        double turnPower = turnPID.calculate(-error);

        drivebase.driveFieldCentric(cX, cY, turnPower, currentHeading);
    }
}
