package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

public class MecanumDriveBase {
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public GoBildaPinpointDriver odo;

    double currentForwardPower = 0.0;
    Double targetHeading = 0.0;

    public MecanumDriveBase(HardwareMap map) {
        // Initialize the motors. Change motor names if necessary.
        frontLeft = map.get(DcMotorEx.class, "FrontLeft");
        frontRight = map.get(DcMotorEx.class, "FrontRight");
        backLeft = map.get(DcMotorEx.class, "BackLeft");
        backRight = map.get(DcMotorEx.class, "BackRight");

        // Reverse motors if necessary, set zero-power behavior, and
        // set run modes
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        // Set up the GoBilda PinPoint Odometry computer for odometry use.
        // Here, we have it setup to use the GoBilda 4-bar odometry pods, with
        // encoder directions set to forward (change if necessary for your own
        // robot), and then it recalibrates itself.
        odo = map.get(GoBildaPinpointDriver.class, "odo");
        odo.initialize();
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.recalibrateIMU();
    }

    /// Functions to be run every loop. For now, we are only updating the odometry.
    public void periodic() {
        odo.update();
    }

    /// Drive using robot-centric mode. SolversLib does implement this themselves, but I would like
    /// to have some manual control over what is going on for when I add my own funcitonality later.
    /// @param x The x-value, normally taken from the controller joystick inputs, to use to drive.
    /// @param y The y-value, normally taken from the controller joystick inputs, to use to drive.
    /// @param a The angle value, normally taken from the controller joystick inputs,
    ///          to use to rotate the robot.
    /// @param heading TODO: Add future lock-to-heading functionality.
    /// TODO: Add forward slewing functionality.
    public void Drive(double x, double y, double a, double heading) {
        double powerDifference = y - currentForwardPower;

        if (powerDifference > DriveConstants.MAX_POWER_STEP) {
            powerDifference = DriveConstants.MAX_POWER_STEP;
        } else if (powerDifference < -DriveConstants.MAX_POWER_STEP) {
            powerDifference = -DriveConstants.MAX_POWER_STEP;
        }

        currentForwardPower += powerDifference;

        double turnPower;
        if (targetHeading != null) {
            double currentHeading = odo.getHeading(AngleUnit.DEGREES);
            double headingError = AngleUnit.normalizeDegrees(targetHeading - currentHeading);

            if (Math.abs(headingError) <= DriveConstants.HEADING_KP) {
                turnPower = 0.0;
            } else {
                turnPower = headingError * DriveConstants.HEADING_KP;
            }

            turnPower = Range.clip(turnPower, -1.0, 1.0);
        } else {
            turnPower = a;
        }

        double denominator = Math.max(Math.abs(currentForwardPower) + Math.abs(x) + Math.abs(a), 1);
        frontLeft.setPower((currentForwardPower + x + a) / denominator);
        frontRight.setPower((currentForwardPower - x - a) / denominator);
        backLeft.setPower((currentForwardPower - x + a) / denominator);
        backRight.setPower((currentForwardPower + x - a) / denominator);
    }

    /// Drive using field-centric mode. SolversLib does implement this themselves, but I would like
    /// to have some manual control over what is going on for when I add my own funcitonality later.
    /// @param x The x-value, normally taken from the controller joystick inputs, to use to drive.
    /// @param y The y-value, normally taken from the controller joystick inputs, to use to drive.
    /// @param a The angle value, normally taken from the controller joystick inputs,
    ///          that you want the robot to face
    public void DriveFieldCentric(double x, double y, double a) {
        double botHeading = odo.getHeading(AngleUnit.RADIANS);

        double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(a), 1);
        frontLeft.setPower((rotatedY + rotatedX + a) / denominator);
        frontRight.setPower((rotatedY - rotatedX - a) / denominator);
        backLeft.setPower((rotatedY - rotatedX + a) / denominator);
        backRight.setPower((rotatedY + rotatedX - a) / denominator);
    }

    /// Drive using field-centric mode with an additional heading lock.
    /// @param gpLX The x-value, normally taken from the controller joystick inputs, to use to drive.
    /// @param gpLY The y-value, normally taken from the controller joystick inputs, to use to drive.
    /// @param gpRX The angle you want the robot to be set to, in degrees.
    public void DriveFieldCentricWithLock(double gpLX, double gpLY, double gpRX, double gpRY) {
        double rotStickMagnitude = Math.hypot(gpRX, gpRY);

        if (rotStickMagnitude > 0.25) {
            double stickAngleRadians = Math.atan2(gpRX, gpRY);
            double stickAngleDegrees = Math.toDegrees(stickAngleRadians);

            this.targetHeading = (stickAngleDegrees + 540) % 360;
        }

        double currentHeadingDegrees = odo.getHeading(AngleUnit.DEGREES);
        double headingError = ((this.targetHeading - currentHeadingDegrees) + 540) % 360;
        double a = Range.clip(headingError * DriveConstants.HEADING_KP, -DriveConstants.MAX_HEADING_CORRECTION_SPEED, DriveConstants.MAX_HEADING_CORRECTION_SPEED);

        double botHeadingRadians = Math.toRadians(currentHeadingDegrees);
        double rotatedX = gpLX * Math.cos(-botHeadingRadians) - gpLY * Math.sin(-botHeadingRadians);
        double rotatedY = gpLX * Math.sin(-botHeadingRadians) + gpLY * Math.cos(-botHeadingRadians);

        double denominator = Math.max(Math.abs(rotatedX) + Math.abs(rotatedY) + Math.abs(a), 1.0);
        frontLeft.setPower((rotatedY + rotatedX + a) / denominator);
        frontRight.setPower((rotatedY - rotatedX - a) / denominator);
        backLeft.setPower((rotatedY - rotatedX + a) / denominator);
        backRight.setPower((rotatedY + rotatedX - a) / denominator);
    }

    public void setTargetHeading(double heading) {
        this.targetHeading = heading;
    }
}
