package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

/**
 * The subsystem class for our intake. It's a relatively simple one,
 * containing only one motor and a function to set its power.
 */
public class IntakeSubsystem extends SubsystemBase {
    // Object that references the intake motor on the robot.
    private final MotorEx intakeMotor;

    /**
     * Initialize the IntakeSubsystem.
     * @param map Uses the HardwareMap from your Auto/TeleOp to intiailize all of the hardware.
     */
    public IntakeSubsystem(HardwareMap map) {
        // Initialize the intake motor.
        intakeMotor = new MotorEx(map, "IntakeMotor");
        intakeMotor.setInverted(true);

        // Important! We want to make sure that we are able to only set the raw
        // power of the intake motor. We don't want any velocity or position
        // control.
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
    }

    /**
     * Sets the intake motor power to a specified double.
     * @param power The double to set the motor power to, ranging from -1 to 1.
     */
    public void setIntakePower(double power) {
        intakeMotor.set(power);
    }
}
