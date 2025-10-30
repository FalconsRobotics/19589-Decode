package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

/// A constants class just for having all the "tuneable" values
/// related to the shooter in one place for simplicity.
@Config
public class ShooterConstants {
    public static double HIGH_RPM_RANGE = 5500; // in RPMs
    public static double LOW_RPM_RANGE = 5000; // in RPMS
    public static double FIRING_SPEED = 5250;

    public static double MOTOR_RPMS = 28; // Taken from GoBilda, from a 6000RPM motor.

    // PID Values, tune as necessary
    public static double kP = 5;
    public static double kI = 5;
    public static double kD = 5;
    public static double kF = 5;
}
