package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CarouselPosition {
    public static double gearRatio = 36/40;

    /// Values tuned if 1st pickup is 'zeroed' at servo position 0.2
    public static double inputMin = 0.177;
    public static int counterMin = -3;
    public static int counterMax = 11;

    public static double servoPosition(int input) {
        double stepSize = (0.4 - 0.2) / 3.0;
        return (inputMin + (input * stepSize));
    }


    public static double distanceMax = 62;

    public static double P_VALUE = 0.3;
}
