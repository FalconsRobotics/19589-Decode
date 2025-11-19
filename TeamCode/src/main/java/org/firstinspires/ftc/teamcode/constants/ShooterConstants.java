package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    // The encoder resolution of a GoBilda 6000RPM motor.
    public static double MOTOR_RESOLUTION = 28;

    public static double VELO_KP = 0.0;
    public static double VELO_KI = 0.0;
    public static double VELO_KD = 0.0;
    public static double VELO_KF = 0.0;
    public static double VELO_MAX_INTEG_ERROR = 5.0;

    public static double SPEED = 4500;
    public static double SPEED_TOLERANCE = 5; // A max tolerance of ± 5tps (± 10rpm)
}
