package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class IntakeElevatorSubsystem extends SubsystemBase {
    // The motor object for the single motor that controlls the
    // intake and elevator.
    private final MotorEx intakeMotor;
    public boolean isIntakeActive = false;

    public IntakeElevatorSubsystem(HardwareMap map) {
        intakeMotor = map.get(MotorEx.class, "IntakeMotor");
        intakeMotor.setInverted(true);
    }

    /// Sets the intake to a given speed.
    /// @param power A double, representing the motor power you want
    ///              from -1 to 1.
    public void setPower(double power) {
        intakeMotor.set(power);
        isIntakeActive = (power != 0.0);
    }
}