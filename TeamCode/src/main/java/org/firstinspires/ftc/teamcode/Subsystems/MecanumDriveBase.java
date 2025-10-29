package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveBase {
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public GoBildaPinpointDriver odo;

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

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

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
    public void Drive(double x, double y, double a, double heading) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(a), 1);
        frontLeft.setPower((y + x + a) / denominator);
        frontRight.setPower((y - x + a) / denominator);
        backLeft.setPower((y - x - a) / denominator);
        backRight.setPower((y + x - a) / denominator);
    }

    /// Drive using field-centric mode. SolversLib does implement this themselves, but I would like
    /// to have some manual control over what is going on for when I add my own funcitonality later.
    /// @param x The x-value, normally taken from the controller joystick inputs, to use to drive.
    /// @param y The y-value, normally taken from the controller joystick inputs, to use to drive.
    /// @param a The angle value, normally taken from the controller joystick inputs,
    ///          to use to rotate the robot.
    /// @param heading TODO: Add future lock-to-heading functionality.
    public void DriveFieldCentric(double x, double y, double a, double heading) {
        double botHeading = odo.getHeading(AngleUnit.RADIANS);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(a), 1);

        double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        frontLeft.setPower((rotatedY + rotatedX + a) / denominator);
        frontRight.setPower((rotatedY - rotatedX + a) / denominator);
        backLeft.setPower((rotatedY - rotatedX - a) / denominator);
        backRight.setPower((rotatedY + rotatedX - a) / denominator);
    }
}
