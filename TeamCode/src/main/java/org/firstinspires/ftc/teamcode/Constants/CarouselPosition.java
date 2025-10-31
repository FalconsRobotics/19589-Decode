package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CarouselPosition {
    public static double gearRatio = 0.9;

    /// Values tuned if 1st pickup is 'zeroed' at servo position 0.2
    public static double intakeMin = 0.2; // 0.185
    public static double halfwayMin = 0.185;
    public static double test = 0.2 / 6;
    public static double startPos = intakeMin + test;
    public static double zeroPos = 0.22;
    public static int counterMin = -3;
    public static int counterMax = 11;

    public static double servoPosition(int input) {
        double stepSize = 0.2 / 3.0;
        return (intakeMin + (input * stepSize));
    }

    public static double halfwayPosition(int input) {
        double stepSize = 0.2 / 3.0;
        return (halfwayMin + (input * stepSize));
    }


    public static double distanceMax = 62;

    public static double P_VALUE = 0.3;
}
