package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;

/**
 * The subsystem class for our drivebase. Holds motor objects for each wheel,
 * objects for our GoBildaPinpoint IMU computer, a PIDF controller for robot
 * rotation, and functions for driving the robot in various modes.
 */
public class DrivebaseSubsystem extends SubsystemBase {
    // Motor objects that store the references to the motors on the robot.
    public final MotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // This is the MecanumDrive built into SolversLib. We may not need to use this at all, as the
    // drive code is all done manually.
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

    public double angleError, targetAngle, currentAngle, calculatedDerivative;

    public double frontLeftPower, frontRightPower, backLeftPower, backRightPower;

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
//        odo.recalibrateIMU();
//        odo.resetPosAndIMU();

        pidTimer = new ElapsedTime();
    }

    /**
     * Reset the heading PID in cases of something being *really* messed up. You shouldn't really
     * have to use this.
     */
    public void resetHeadingPID() {
        // Set the last angle error to 0.
        this.lastAngleError = 0.0;

        // Reset the timer to 0.
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

        // Finally, set the motor powers!
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

        // Set our motor powers!
        frontLeftMotor.set(limitedForward + robotStrafe + cA);
        frontRightMotor.set(limitedForward - robotStrafe - cA);
        backLeftMotor.set(limitedForward - robotStrafe + cA);
        backRightMotor.set(limitedForward + robotStrafe - cA);
    }

    /**
     * Drives the robot field-centrically and locks to an angle.
     * @param cX The x-value taken from the gamepad, used for strafing.
     * @param cY The y-value taken from the gamepad, used for forward/backward movement.
     * @param targetAngleDeg A custom heading to lock the drivebase to.
     */
    public void driveFieldCentricHeadingLock(double cX, double cY, double targetAngleDeg) {
        /// Obtain Necessary Values
        // Get the robot's current heading in radians, necessary for the trig functions
        // that make field-centric driving possible.
        double headingRad = odo.getHeading(AngleUnit.RADIANS);
        // Get the robot's current heading in degrees, which is just taken from our existing headingRad.
        double headingDeg = Math.toDegrees(headingRad);

        double headingNormalized02pi = odo.getHeading(UnnormalizedAngleUnit.RADIANS) % (2 * Math.PI);

        // Transform the desired field-centric movements into robot-centric ones, accounting
        // for the robot's current orientation on the field.
        double forwardPower = cY * Math.cos(headingNormalized02pi) - cX * Math.sin(headingNormalized02pi);
        double strafePower = cY * Math.sin(headingNormalized02pi) + cX * Math.cos(headingNormalized02pi);

        /// Slew Rate Limiter
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

        /// Heading control
        // Calculate the difference between the current heading and the target angle (targetAngleDeg).
        double turnPower;

        // Normalize the angle to keep it between -180 and 180 degrees.
        // TODO: Make sure this is the correct formula.
        angleError = ((targetAngleDeg - headingDeg + 180) % 360 + 360) % 360 - 180;

        // Take the time since the last loop and store it in deltaTime, then reset the timer for an
        // accurate calculation in the next frame.
        double deltaTime = pidTimer.seconds();
        pidTimer.reset();

        // Calculate the desired turn power by using a simple P controller.
        // The error is multiplied by a tuning constant (kP) to determine the motor turning power.
        double pTerm = angleError * DriveConstants.DRIVE_KP;

        // Calculate the D term of the PD controller.
        double derivative = (angleError - this.lastAngleError) / deltaTime;
        double dTerm = derivative * DriveConstants.DRIVE_KD;

        // Store the current angle error in a local variable (after we've already used it) in order to use it in the next frame.
        this.lastAngleError = angleError;

        // We only want to set the turnPower if we're not within the heading tolerance.
        // The actual calculations still need to be calculated, which is why they're outside of this.
        if (headingDeg > targetAngleDeg - DriveConstants.HEADING_TOLERANCE &&
            headingDeg < targetAngleDeg + DriveConstants.HEADING_TOLERANCE) {
            turnPower = 0.0;
        } else { // If we are within that tolerance, then we don't want to turn. Hence, turnPower = 0.0.
            // Store the total calculated power from the our homemade PD controller in turnPower.
            turnPower = pTerm + dTerm;
        }

        /// Calculating motor powers
        // Store the powers for each wheel in its own variable for easy use.
        // There is some specific math for doing this, as when turning, the Mecanum wheels essentially
        // acts like a tank drive.
        this.frontLeftPower = limitedForward + strafePower + turnPower;
        this.frontRightPower = limitedForward - strafePower - turnPower;
        this.backLeftPower = limitedForward - strafePower + turnPower;
        this.backRightPower = limitedForward + strafePower - turnPower;

        // Normalize the motor powers to keep them between -1.0 to 1.0.
        double maxPower = Math.max(
                Math.max(
                        Math.abs(frontLeftPower),
                        Math.abs(frontRightPower)
                ),
                Math.max(
                        Math.abs(backLeftPower),
                        Math.abs(backRightPower)
                )
        );

        // If the motor powers are greater than 1, divide them by that maxPower to normalize them.
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Finally, set the motor powers! Our robot drives!
        frontLeftMotor.set(frontLeftPower);
        frontRightMotor.set(frontRightPower);
        backLeftMotor.set(backLeftPower);
        backRightMotor.set(backRightPower);

        /// Store basic variables for telemetry reporting.
        this.currentAngle = headingDeg;
        this.targetAngle = targetAngleDeg;
        this.calculatedDerivative = derivative;
    }

    /**
     * Normalize an angle to be between -180 and 180 degrees.
     * @param deg Angle (in degrees) to normalize between -180 and 180.
     * @return Returns the angle between -180 and 180.
     */
    public double normalizeTo180Deg(double deg) {
        return ((deg + 180) % 360 + 360) % 360 - 180;
    }
}
