package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase {
    private final DcMotorEx extakeMotor;

    // TODO: Tune motor speeds
    public static final class ExtakeSpeeds {
        public static final double BACK_TRIANGLE = 1.0;
        public static final double CLOSE_TRIANGLE = 0.5;
        public static final double STOPPED = 0.0;
    }

    public ShooterSubsystem(HardwareMap map) {
        extakeMotor = map.get(DcMotorEx.class, "ExtakeMotor");
        extakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void setPower(double speed) { extakeMotor.setPower(speed); }
}
