package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {
    // Constants used to tune the heading lock functionality.
    public static double DRIVE_KP = -0.04167;
    public static double DRIVE_KD = -0.003;

    // Constant used to limit the maximum acceleration to prevent tipping.
    public static double MAX_ACCEL_CHANGE = 0.05;

    // The difference in heading that the robot will consider to be "lined up".
    public static double HEADING_TOLERANCE = 5;
}