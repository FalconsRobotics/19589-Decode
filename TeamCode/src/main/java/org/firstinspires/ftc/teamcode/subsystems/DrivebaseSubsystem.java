package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;

/**
 * The subsystem class for our drivebase. Holds motor objects for each wheel,
 * objects for our GoBildaPinpoint IMU computer, a PIDF controller for robot
 * rotation, and functions for driving the robot in various modes.
 */
public class DrivebaseSubsystem extends SubsystemBase {
    // Motor objects that store the references to the motors on the robot.
    public final MotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // This is the MecanumDrive built into SolversLib, and we'll be relying on its
    // built-in functions.
    public final MecanumDrive drivebase;

    // Object to store the GoBildaPinpointDriver IMU.
    public final GoBildaPinpointDriver odo;

    // Object to hold a PID controller that we will use to rotate the robot.
    public final PIDFController turnPID;

    // A private variable that tracks our acceleration across OpMode frames. Used
    // for a slew rate controller to prevent our robot from tipping when moving
    // in the forward/background direction.
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

        odo.setOffsets(80.0, 96.0, DistanceUnit.MM);

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

    @Override
    public void periodic() {
        odo.update();
    }

    /**
     * Calls the SolversLib MecanumDrive driveRobotCentric() function. Uses a slew rate
     * controller to make sure that our robot doesn't tip.
     * @param cX The x-value taken from the gamepad, used for strafing.
     * @param cY The y-value taken from the gamepad, used for forward/backward movement.
     * @param cA The a-value taken from the gamepad, used for rotation.
     */
    public void drive(double cX, double cY, double cA) {
        // Calculate the difference between the current motor power
        double change = cY - this.lastForwardMovement;

        // Keep the acceleration between -MAX_ACCEL_CHANGE and +MAX_ACCEL_CHANGE, a variable
        // that is defined in DriveConstants so that it can be easily manipulated.
        if (change > DriveConstants.MAX_ACCEL_CHANGE) {
            change = DriveConstants.MAX_ACCEL_CHANGE;
        } else if (change < -DriveConstants.MAX_ACCEL_CHANGE) {
            change = -DriveConstants.MAX_ACCEL_CHANGE;
        }

        // Add/subtract the limited acceleration from our last forward/background movement.
        double limitedForward = this.lastForwardMovement + change;

        // Call the SolversLib robot-centric driving function using the supplied controller values
        // and the forward/background movement with our new slew controller.
        drivebase.driveRobotCentric(cX, limitedForward, cA);

        // Store our last forward/backward speed in a class member so that we can
        // track how quickly we're changing acceleration across the OpMode loops.
        this.lastForwardMovement = limitedForward;
    }

    /**
     * Calls the SolversLib MecanumDrive driveFieldCentric() function. Relies on the
     * built-in GoBildaPinpoint heading rather than passing one in. Uses a slew rate
     * controller to make sure that our robot doesn't tip.
     * @param cX The x-value taken from the gamepad, used for strafing.
     * @param cY The y-value taken from the gamepad, used for forward/backward movement.
     * @param cA The a-value taken from the gamepad, used for rotation.
     */
    public void driveFieldCentric(double cX, double cY, double cA) {
        // odo.getHeading() retrieves the current heading from the Pinpoint computer.
        // The SolversLib function requires degrees, so that's what we'll use.
        double headingRad = odo.getHeading(AngleUnit.RADIANS);

        // Calculate the unit vectors for our field-centric driving.
        double robotForward = cY * Math.cos(headingRad) + cX * Math.sin(headingRad);
        double robotStrafe = cY * -Math.sin(headingRad) + cX * Math.cos(headingRad);

        // Keep the acceleration between -MAX_ACCEL_CHANGE and +MAX_ACCEL_CHANGE, a variable
        // that is defined in DriveConstants so that it can be easily manipulated.
        double change = robotForward - this.lastForwardMovement;
        if (change > DriveConstants.MAX_ACCEL_CHANGE) {
            change = DriveConstants.MAX_ACCEL_CHANGE;
        } else if (change < -DriveConstants.MAX_ACCEL_CHANGE) {
            change = -DriveConstants.MAX_ACCEL_CHANGE;
        }

        // Add/subtract the limited acceleration from our last forward/background movement.
        double limitedForward = this.lastForwardMovement + change;

        // Call the SolversLib field-centric driving function using the supplied controller values
        // and the forward/background movement with our new slew controller.
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
        // odo.getHeading() retrieves the current heading from the Pinpoint computer.
        // The SolversLib function requires degrees, so that's what we'll use.
        double headingRad = odo.getHeading(AngleUnit.RADIANS);

        // Calculate the unit vectors for our field-centric driving.
        double robotForward = cY * Math.cos(headingRad) + cX * Math.sin(headingRad);
        double robotStrafe = cY * -Math.sin(headingRad) + cX * Math.cos(headingRad);

        // Keep the acceleration between -MAX_ACCEL_CHANGE and +MAX_ACCEL_CHANGE, a variable
        // that is defined in DriveConstants so that it can be easily manipulated.
        double change = robotForward - this.lastForwardMovement;
        if (change > DriveConstants.MAX_ACCEL_CHANGE) {
            change = DriveConstants.MAX_ACCEL_CHANGE;
        } else if (change < -DriveConstants.MAX_ACCEL_CHANGE) {
            change = -DriveConstants.MAX_ACCEL_CHANGE;
        }

        // Add/subtract the limited acceleration from our last forward/background movement.
        double limitedForward = this.lastForwardMovement + change;

        // Pull our current heading in degrees from the Pinpoint
        double currentHeading = odo.getHeading(AngleUnit.DEGREES);

        // Calculate the error between the desired heading and the current heading, limit
        // it from -180 to 180 degrees, and calculate the turn power required to turn to
        // that heading using our PIDF controller.
        double error = AngleUnit.normalizeDegrees(lockHeading - currentHeading);
        double turnPower = turnPID.calculate(-error);

        // Call the SolversLib field-centric driving function with the calculated turn power
        // and slew rate controller.
        drivebase.driveFieldCentric(cX, limitedForward, turnPower, currentHeading);

        this.lastForwardMovement = limitedForward;
    }
}
