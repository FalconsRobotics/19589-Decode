package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
//import com.seattlesolvers.solverslib.controller.PIDFController;
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

    // A private variable that tracks our acceleration across OpMode frames. Used
    // for a slew rate controller to prevent our robot from tipping when moving
    // in the forward/background direction.
    private double lastForwardMovement = 0.0;

    // A private variable that stores the time, used for calculating the delta loop time, which
    // is then used for calculating the d-term in a PD controller.
    private ElapsedTime pidTimer;

    private double lastAngleError = 0.0;

    public double angleError, targetAngle, currentAngle;

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

        frontLeftMotor.setInverted(true);
        backLeftMotor.setInverted(true);

        // Create the SolversLib MecanumDrive object from those four motors.
        drivebase = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        // Pull the IMU from the HardwareMap and initialize it with the proper settings,
        // using the GoBilda 4-bar odometry pod resolution and forward directions.
        odo = map.get(GoBildaPinpointDriver.class, "odo");
        odo.initialize();
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set the distances of the x and y encoders from the odometry module, used for accurately computing heading.
        odo.setOffsets(80.0, 96.0, DistanceUnit.MM);

        // Finally, once everything is setup, we need to tell the IMU to zero itself.
        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        pidTimer = new ElapsedTime();
    }

    public void resetHeadingPID() {
        this.lastAngleError = 0.0;
        this.pidTimer.reset();
    }

    /**
     * These are functions that need to be called every loop of our OpModes.
     */
    @Override
    public void periodic() {
        odo.update();
    }

    /**
     * Drives robot-centrically. Uses a slew rate controller to make sure that our robot doesn't tip.
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

        // Store our last forward/backward speed in a class member so that we can
        // track how quickly we're changing acceleration across the OpMode loops.
        this.lastForwardMovement = limitedForward;

        frontLeftMotor.set(limitedForward + cX + cA);
        frontRightMotor.set(limitedForward - cX - cA);
        backLeftMotor.set(limitedForward - cX + cA);
        backRightMotor.set(limitedForward + cX - cA);
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
        double robotStrafe = -cY * Math.sin(headingRad) + cX * Math.cos(headingRad);

        // Keep the acceleration between -MAX_ACCEL_CHANGE and +MAX_ACCEL_CHANGE, a variable
        // that is defined in DriveConstants so that it can be easily manipulated.
        double forwardChange = robotForward - this.lastForwardMovement;
        if (forwardChange > DriveConstants.MAX_ACCEL_CHANGE) {
            forwardChange = DriveConstants.MAX_ACCEL_CHANGE;
        } else if (forwardChange < -DriveConstants.MAX_ACCEL_CHANGE) {
            forwardChange = -DriveConstants.MAX_ACCEL_CHANGE;
        }

        // Add/subtract the limited acceleration from our last forward/background movement.
        double limitedForward = this.lastForwardMovement + forwardChange;
        this.lastForwardMovement = limitedForward;

        // Call the SolversLib field-centric driving function using the supplied controller values
        // and the forward/background movement with our new slew controller.
        frontLeftMotor.set(limitedForward + robotStrafe + cA);
        frontRightMotor.set(limitedForward - robotStrafe - cA);
        backLeftMotor.set(limitedForward - robotStrafe + cA);
        backRightMotor.set(limitedForward + robotStrafe - cA);
    }

    /**
     * Drives the robot field-centrically and locks to an angle.
     * @param cX The x-value taken from the gamepad, used for strafing.
     * @param cY The y-value taken from the gamepad, used for forward/backward movement.
     * @param lockHeading A custom heading to lock the drivebase to.
     */
    public void driveFieldCentricHeadingLock(double cX, double cY, double lockHeading) {
        // Get the robot's current heading in radians, necessary for the trig functions
        // that make field-centric driving possible.
        double headingRad = odo.getHeading(AngleUnit.RADIANS);

        // Transform the desired field-centric movements into robot-centric ones, accounting
        // for the robot's current orientation on the field.
        double forwardPower = cY * Math.cos(headingRad) + cX * Math.sin(headingRad);
        double strafePower = -cY * Math.sin(headingRad) + cX * Math.cos(headingRad);

        // Slew rate limiter:
        // Find the change in the forward/background movement from OpMode loops.
        double change = forwardPower - this.lastForwardMovement;
        // Keep the change per loop between -MAX_ACCEL_CHANGE and +MAX_ACCEL_CHANGE, which
        // are just constants defined in DriveConstants that define the max step.
        if (change > DriveConstants.MAX_ACCEL_CHANGE) { change = DriveConstants.MAX_ACCEL_CHANGE; }
        else if (change < -DriveConstants.MAX_ACCEL_CHANGE) { change = -DriveConstants.MAX_ACCEL_CHANGE; }

        // Apply the newly-calculated limited change to our forward movement.
        double limitedForward = this.lastForwardMovement + change;
        // Store the current limitedForward so that we can check it again in the next loop.
        this.lastForwardMovement = limitedForward;

        // Heading control:
        // Get the robot's current heading in degrees, which is just taken from our existing
        // headingRad.
        double headingDeg = Math.toDegrees(headingRad);

        // Calculate the difference between the current heading and the target angle (lockHeading).
        double angleError = lockHeading - headingDeg;

        // Normalize the angle to keep it between -180 and 180 degrees.
        angleError %= 360;
        if (angleError > 180) { angleError -= 360; }
        else if (angleError < -180) { angleError += 360; }
        this.lastAngleError = angleError;

        double deltaTime = pidTimer.seconds();
        pidTimer.reset();

        // Calculate the desired turn power by using a simple P controller.
        // The error is multiplied by a tuning constant (kP) to determine the motor turning power.
        double pTerm = angleError * DriveConstants.DRIVE_KP;

        double derivative = (angleError - lastAngleError) / deltaTime;
        double dTerm = derivative * DriveConstants.DRIVE_KD;

        double turnPower = pTerm + dTerm;

        double frontLeftPower = limitedForward + strafePower + turnPower;
        double frontRightPower = limitedForward - strafePower - turnPower;
        double backLeftPower = limitedForward - strafePower + turnPower;
        double backRightPower = limitedForward + strafePower - turnPower;

        // Normalize the motor powers to keep them between -1.0 to 1.0.
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeftMotor.set(frontLeftPower);
        frontRightMotor.set(frontRightPower);
        backLeftMotor.set(backLeftPower);
        backRightMotor.set(backRightPower);
    }
}
