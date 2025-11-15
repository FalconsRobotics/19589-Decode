package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    // Object used to store the motor reference.
    MotorEx shooterMotor;

    /**
     * Initialize the ShooterSubsystem.
     * @param map Uses the HardwareMap from your Auto/TeleOp to initialize all the hardware.
     */
    public ShooterSubsystem(HardwareMap map) {
        shooterMotor = new MotorEx(map, "ShooterMotor");
        shooterMotor.setInverted(true);
        shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        shooterMotor.setVeloCoefficients(ShooterConstants.VELO_KP, ShooterConstants.VELO_KI, ShooterConstants.VELO_KD);
    }

    /**
     * Sets the velocity of the shooter motor using the SolversLib PID controller that's built in
     * to the MotorEx class.
     * @param speed The motor, in revolutions per minute (rpm), that the motor should rotate at.
     */
    public void setVelocity(double speed) {
        shooterMotor.setVelocity(speed * 60 / ShooterConstants.MOTOR_RESOLUTION);
    }

    /**
     * Gets the current velocity of the shooter motor in revolutions per second (rpm).
     * @return Returns the current velocity of the shooter motor in revolutions per second (rpm).
     */
    public double getVelocity() {
        return shooterMotor.getVelocity() * 60 / ShooterConstants.MOTOR_RESOLUTION;
    }
}
