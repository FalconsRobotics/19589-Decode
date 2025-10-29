package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

/// A constants class just for having all the "tuneable" values
/// related to the shooter in one place for simplicity.
@Config
public class ShooterConstants {
    public static final double HIGH_RPM_RANGE = 5500; // in RPMs
    public static final double LOW_RPM_RANGE = 5000; // in RPMS
    public static final double FIRING_SPEED = 5250;

    public static final double MOTOR_RPMS = 28; // Taken from GoBilda, from a 6000RPM motor.

    // PID Values, tune as necessary
    public static final double kP = 0.005;
    public static final double kI = 0.001;
    public static final double kD = 0.01;
    public static final double kF = 0.015;
}
