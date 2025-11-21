package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    // The encoder resolution of a GoBilda 6000RPM motor.
    public static double MOTOR_RESOLUTION = 28;

    public static double VELO_KP = 100.0;
    public static double VELO_KI = 0.0;
    public static double VELO_KD = 0.0;
    public static double VELO_KF = 14.5;
    public static double VELO_MAX_INTEG_ERROR = 5.0;

    public static double BLUE_FAR_SPEED = 2080.0 * 60.0 / 28.0;
    public static double BLUE_CLOSE_SPEED = 1580.0 * 60.0 / 28.0;
    public static double RED_FAR_SPEED = 2060.0 * 60.0 / 28.0;
    public static double RED_CLOSE_SPEED = 1600.0 * 60.0 / 28.0;
    public static double SPEED_TOLERANCE = 5; // A max tolerance of ± 5tps (± 10.7rpm)
}
