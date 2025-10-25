package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveBase extends SubsystemBase {
    public MotorEx frontLeft, frontRight, backLeft, backRight;
    public GoBildaPinpointDriver odo;

    public MecanumDriveBase(HardwareMap map) {
        // Initialize the motors. Change motor names if necessary.
        frontLeft = map.get(MotorEx.class, "FrontLeft");
        frontRight = map.get(MotorEx.class, "FrontRight");
        backLeft = map.get(MotorEx.class, "BackLeft");
        backRight = map.get(MotorEx.class, "BackRight");

        // Set up the GoBilda PinPoint Odometry computer for odometry use.
        // Here, we have it setup to use the GoBilda 4-bar odometry pods, with
        // directions set to forward (change if necessary for your own robot),
        // and then it recalibrates itself.
        odo = map.get(GoBildaPinpointDriver.class, "odo");
        odo.initialize();
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.recalibrateIMU();
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
        frontLeft.set((y + x + a) / denominator);
        frontRight.set((y - x + a) / denominator);
        backLeft.set((y - x - a) / denominator);
        backRight.set((y + x - a) / denominator);
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

        frontLeft.set((rotatedY + rotatedX + a) / denominator);
        frontRight.set((rotatedY - rotatedX + a) / denominator);
        backLeft.set((rotatedY - rotatedX - a) / denominator);
        backRight.set((rotatedY + rotatedX - a) / denominator);
    }
}
