package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class ShooterSubsystem extends SubsystemBase {
    private final MotorEx extakeMotor;

    // TODO: Tune motor speeds
    public static final class ExtakeSpeeds {
        public static final double BACK_TRIANGLE = 1.0;
        public static final double CLOSE_TRIANGLE = 0.5;
        public static final double STOPPED = 0.0;
    }

    public ShooterSubsystem(HardwareMap map) {
        extakeMotor = map.get(MotorEx.class, "ExtakeMotor");
        extakeMotor.setInverted(true);
    }

    public void setPower(double speed) { extakeMotor.set(speed); }
}
