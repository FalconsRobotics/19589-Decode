package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    // Object used to store the motor reference.
    public DcMotorEx shooterMotor;

    // Something that we can use to track the desired speed across loops.
    public double targetSpeed = 0.0;

    // Variables for storing measured values for setting the motor velocity with PID.
    ElapsedTime loopTimer;
    public double lastVelocityError = 0.0;
    public double currentVelocity = 0.0;
    private double shooterTicks;
    private double shooterRPM;
    private double lastShooterTicks;

    /**
     * Initialize the ShooterSubsystem.
     * @param map Uses the HardwareMap from your Auto/TeleOp to initialize all the hardware.
     */
    public ShooterSubsystem(HardwareMap map) {
        shooterMotor = map.get(DcMotorEx.class, "ShooterMotor");

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        loopTimer = new ElapsedTime();
    }

    /**
     * Sets the velocity of the shooter motor using a PD controller, in rpm.
     * @param speed The motor, in revolutions per minute (rpm), that the motor should rotate at.
     */
    public void setVelocity(double speed) {
//        shooterTicks = shooterMotor.getCurrentPosition();
//        shooterRPM = ((shooterTicks - lastShooterTicks) / loopTimer.milliseconds()) * (60 / ShooterConstants.MOTOR_RESOLUTION);
//        lastShooterTicks = shooterTicks;
//
//        double velocityError = speed - shooterRPM;
//
//        double shooterPTerm = velocityError * ShooterConstants.VELO_KP;
//        double shooterDTerm = ((velocityError - this.lastVelocityError) / loopTimer.milliseconds()) * ShooterConstants.VELO_KD;
//        double shooterFTerm;
//
//        this.lastVelocityError = velocityError;
//
//        if (Math.abs(velocityError) > 200) {
//            shooterFTerm = ShooterConstants.VELO_KF;
//        } else {
//            shooterFTerm = 0.0;
//        }
//
//        double motorPower = shooterPTerm + shooterDTerm + shooterFTerm;
//        shooterMotor.setPower(motorPower);
//
//        loopTimer.reset();

        shooterMotor.setVelocity(speed * ShooterConstants.MOTOR_RESOLUTION / 60);
    }

    /**
     * Returns whether the shooter motor is at the desired speed within a given tolerance constant.
     * @return Returns whether the shooter motor is at the desired speed within a given tolerance constant.
     */
    public boolean isInTargetSpeed() {
        return Math.abs(this.lastVelocityError) <= ShooterConstants.SPEED_TOLERANCE;
    }
}
