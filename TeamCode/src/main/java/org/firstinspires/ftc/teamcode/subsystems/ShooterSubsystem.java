package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    // Object used to store the motor reference.
    DcMotorEx shooterMotor;

    // Something that we can use to track the desired speed across loops.
    public double targetSpeed = 0.0;

    // Variables for storing measured values for setting the motor velocity with PID.
    ElapsedTime deltaTimer;
    double lastShooterTicks = 0.0;
    double lastRPMError = 0.0;

    /**
     * Initialize the ShooterSubsystem.
     * @param map Uses the HardwareMap from your Auto/TeleOp to initialize all the hardware.
     */
    public ShooterSubsystem(HardwareMap map) {
        shooterMotor = map.get(DcMotorEx.class, "ShooterMotor");

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        deltaTimer = new ElapsedTime();
    }

    /**
     * Sets the velocity of the shooter motor using the SolversLib PID controller that's built in
     * to the MotorEx class.
     * @param speed The motor, in revolutions per minute (rpm), that the motor should rotate at.
     */
    public void setVelocity(double speed) {
        // Some temporary variables for tracking the velocity for PID.
        double shooterTicks = 0.0;
        double measuredShooterRPM = 0.0;
        double rpmError = 0.0;
        double integError = 0.0;
        double pTerm = 0.0;
        double iTerm = 0.0;
        double dTerm = 0.0;
        double fTerm = 0.0;

        this.targetSpeed = speed;

        if (this.targetSpeed == 0) {
            shooterMotor.setPower(0.0);
        } else {
            shooterTicks = shooterMotor.getCurrentPosition();
            measuredShooterRPM = ((shooterTicks - lastShooterTicks) / deltaTimer.milliseconds()) * (60 / ShooterConstants.MOTOR_RESOLUTION);
            lastShooterTicks = shooterTicks;

            rpmError = measuredShooterRPM - targetSpeed;
            pTerm = rpmError * ShooterConstants.VELO_KP;

            integError += rpmError * deltaTimer.milliseconds() + ShooterConstants.VELO_KI;
            integError = Math.max(-ShooterConstants.VELO_MAX_INTEG_ERROR, Math.min(ShooterConstants.VELO_MAX_INTEG_ERROR, integError));
            iTerm = integError * ShooterConstants.VELO_KI;

            dTerm = ((rpmError - this.lastRPMError) / deltaTimer.milliseconds()) * ShooterConstants.VELO_KD;
            this.lastRPMError = rpmError;

            fTerm = targetSpeed * ShooterConstants.VELO_KF;

            shooterMotor.setPower(
                    Math.max(-1, Math.min(1, pTerm + iTerm + dTerm + fTerm))
            );

            deltaTimer.reset();
        }
    }

    /**
     * Returns whether the shooter motor is at the desired speed within a given tolerance constant.
     * @return Returns whether the shooter motor is at the desired speed within a given tolerance constant.
     */
    public boolean isInTargetSpeed() {
        return Math.abs(this.lastRPMError) <= ShooterConstants.SPEED_TOLERANCE;
    }
}
