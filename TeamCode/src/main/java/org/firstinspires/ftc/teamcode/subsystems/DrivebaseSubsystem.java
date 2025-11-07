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

    private double lastForwardMovement = 0.0;

    /**
     * Initialize the DrivebaseSubsystem.
     * @param map Uses the HardwareMap from your Auto/TeleOp to intiailize all of the hardware.
     */
    public DrivebaseSubsystem(HardwareMap map) {
        // Initialize the four Mecanum wheel motors.
        frontLeftMotor = new MotorEx(map, "FrontLeft");
        frontRightMotor = new MotorEx(map, "FrontRight");
        backLeftMotor = new MotorEx(map, "BackLeft");
        backRightMotor = new MotorEx(map, "BackRight");

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
        // These PIDF values are pulled from a seperate Constants file.
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
        // Call the SolversLib robot-centric driving function using the supplied controller values.

        double change = cY - this.lastForwardMovement;

        if (change > DriveConstants.MAX_ACCEL_CHANGE) {
            change = DriveConstants.MAX_ACCEL_CHANGE;
        } else if (change < -DriveConstants.MAX_ACCEL_CHANGE) {
            change = -DriveConstants.MAX_ACCEL_CHANGE;
        }

        double limitedForward = this.lastForwardMovement + change;

        drivebase.driveRobotCentric(cX, limitedForward, cA);

        this.lastForwardMovement = limitedForward;
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

        double headingRad = odo.getHeading(AngleUnit.RADIANS);
        double robotForward = cY * Math.cos(headingRad) + cX * Math.sin(headingRad);
        double robotStrafe = cY * -Math.sin(headingRad) + cX * Math.cos(headingRad);

        double change = robotForward - this.lastForwardMovement;
        if (change > DriveConstants.MAX_ACCEL_CHANGE) {
            change = DriveConstants.MAX_ACCEL_CHANGE;
        } else if (change < -DriveConstants.MAX_ACCEL_CHANGE) {
            change = -DriveConstants.MAX_ACCEL_CHANGE;
        }

        double limitedForward = this.lastForwardMovement + change;

        drivebase.driveFieldCentric(cX, limitedForward, cA, odo.getHeading(AngleUnit.DEGREES));

        this.lastForwardMovement = limitedForward;
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
        // Pull our current heading in degrees from the Pinpoint
        double currentHeading = odo.getHeading(AngleUnit.DEGREES);

        // Calculate the error between the desired heading and the current heading, limit
        // it from -180 to 180 degrees, and calculate the turn power required to turn to
        // that heading.
        double error = AngleUnit.normalizeDegrees(lockHeading - currentHeading);
        double turnPower = turnPID.calculate(-error);

        // Call the SolversLib field-centric driving function with the calculated turn power.
        drivebase.driveFieldCentric(cX, cY, turnPower, currentHeading);
    }
}
