package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ColorDriver {
    private boolean chained;
    private Servo LED1, LED2, LED3;

    public ColorDriver(HardwareMap map, boolean chainedV) {
        this.chained = chainedV;

        this.LED1 = map.get(Servo.class, "led1");
        if (!chainedV) { this.LED2 = map.get(Servo.class, "led2"); this.LED3 = map.get(Servo.class, "led3");}
    }

    public void setLedColor(LED led, double color){
        if (led == LED.CHAINED || led == LED.LED1) {LED1.setPosition(color);}
        else if (led == LED.LED2) {LED2.setPosition(color);}
        else if (led == LED.LED3) {LED3.setPosition(color);}
    }

    public static class Color {
        /// Spectrum: 0.28 - 0.72 ( > 0.73 is White)
        public static double RED = 0.28;
        public static double ORANGE = 0.3;
        public static double YELLOW = 0.34;
        public static double GREEN = 0.5;
        public static double TURQUOISE = 0.51;
        public static double LIGHT_BLUE = 0.52;
        public static double BLUE = 0.62;
        public static double PURPLE = 0.7;
        public static double WHITE = 0.73;

        public static class BALL {
            public static double GREEN = 0.5;
            public static double PURPLE = 0.71;
        }

        public static class TEAM {
            public static double RED = 0.28;
            public static double BLUE = 0.62;
        }
    }
    public static enum LED {
        LED1, LED2, LED3, CHAINED
    }
}
