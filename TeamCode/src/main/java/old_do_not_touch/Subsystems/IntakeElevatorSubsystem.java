package old_do_not_touch.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeElevatorSubsystem {
    // The motor object for the single motor that controlls the
    // intake and elevator.
    private final DcMotorEx intakeMotor;
    public boolean isIntakeActive = false;

    public IntakeElevatorSubsystem(HardwareMap map) {
        intakeMotor = map.get(DcMotorEx.class, "IntakeMotor");
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    /// Sets the intake to a given speed.
    /// @param power A double, representing the motor power you want
    ///              from -1 to 1.
    public void setPower(double power) {
        intakeMotor.setPower(power);
        isIntakeActive = (power != 0.0);
    }
}