package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class IntakeElevatorSubsystem extends SubsystemBase {
    // The motor object for the single motor that controlls the
    // intake and elevator.
    private final MotorEx intakeMotor;

    public IntakeElevatorSubsystem(HardwareMap map) {
        intakeMotor = map.get(MotorEx.class, "IntakeMotor");
        intakeMotor.setInverted(true);
    }

    /// Turns the intake/elevator system on.
    public void turnOn() { intakeMotor.set(1.0); }

    /// Turns the intake/elevator system off.
    public void turnOff() { intakeMotor.set(0.0); }
}
