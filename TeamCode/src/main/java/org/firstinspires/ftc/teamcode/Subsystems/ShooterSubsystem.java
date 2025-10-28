package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class ShooterSubsystem {
    private final MotorEx extakeMotor;

    public ShooterSubsystem(HardwareMap map) {
        extakeMotor = map.get(MotorEx.class, "ExtakeMotor");
        extakeMotor.setInverted(true);
    }

    public void setPower() {
        // TODO: Create PID control for rev control
    }
}
