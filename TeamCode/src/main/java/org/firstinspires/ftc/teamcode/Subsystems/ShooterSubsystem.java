package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;

public class ShooterSubsystem {
    private final DcMotorEx extakeMotor;
    private final RevBlinkinLedDriver ledStrip;

    public ShooterSubsystem(HardwareMap map) {
        // Setup the motor, set its direction correctly.
        extakeMotor = map.get(DcMotorEx.class, "ExtakeMotor");
        extakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ledStrip = map.get(RevBlinkinLedDriver.class, "LEDStrip");

        // For PID, we want the motor to use the encoder, allowing it to
        // use it the built-in velocity control that comes with DcMotorEx.
        // Floating is just specifying that we don't want it to stop when we
        // don't supply a power, which shouldn't ever happen unless
        extakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Rely on the built in DcMotorEx PID controller, feeding it PID
        // tuning that we pull from ShooterConstants.
        extakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        ShooterConstants.kP,
                        ShooterConstants.kI,
                        ShooterConstants.kD,
                        ShooterConstants.kF
                )
        );
    }

    /// Functions to be run every frame.
    public void periodic() {
        setStripColor();
    }

    /// Checks whether the shooter motor is in the correct power
    /// range to shoot a ball properly.
    /// @return Whether the shooter motor is in the correct power range.
    public boolean isInPowerBand() {
        // Pull the velocity (specified in degrees) from the motor encoder,
        // and check whether it's between the RPM range we give it
        double extakeVelocityRPM = extakeMotor.getVelocity(AngleUnit.DEGREES) * 60.0 / ShooterConstants.MOTOR_RPMS;
        return extakeVelocityRPM <= ShooterConstants.HIGH_RPM_RANGE && extakeVelocityRPM >= ShooterConstants.LOW_RPM_RANGE;
    }

    /// PID controller to set the velocity of the motor.
    public void setPower(double speed) {
        // Sets the motor velocity to speed (in RPM), converting the
        // RPM to ticks.
        extakeMotor.setVelocity(speed * 60 / ShooterConstants.MOTOR_RPMS);
    }

    public void setStripColor() {
        ledStrip.setPattern(
                isInPowerBand() ?
                        RevBlinkinLedDriver.BlinkinPattern.GREEN :
                        RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
    }

    /// Returns the motor velcoity in specified units
    /// @returns The velocity of the shooter motor in ticks.
    public double getVelocity() {
        return extakeMotor.getVelocity();
    }

    /// Returns the motor velcoity in specified units
    /// @param unit The AngleUnit you want the velocity to be returned in
    /// @returns The velocity of the shooter motor in the specified units.
    public double getVelocity(AngleUnit unit) {
        return extakeMotor.getVelocity(unit);
    }
}
