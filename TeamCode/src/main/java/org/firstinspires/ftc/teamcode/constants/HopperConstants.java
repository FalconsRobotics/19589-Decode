package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class HopperConstants {
    public static int TICKS_PER_REVOLUTION = 2048;
    public static double TICKS_PER_STEP = (double) TICKS_PER_REVOLUTION / 3.0;
    public static double HOMING_POWER = 0.1;
    public static double MOVE_KP = 0.002;
    public static double MOVE_MAX = 0.5;
    public static int TOLERANCE_TICKS = 20;
    public static double MOVE_MIN = 0.15;
}
