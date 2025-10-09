package org.firstinspires.ftc.teamcode.color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ColorDriver {
    private boolean chained;
    private Servo LED1;
    //LED2, LED3;

    public ColorDriver(HardwareMap map) {
        LED1 = map.get(Servo.class, "led1");
    }

    public void initLED(HardwareMap map) {
        LED1 = map.get(Servo.class, "led1");
    }

    public void setLedColor(LED led, double color) {
        if (led == LED.CHAINED || led == LED.LED1) {LED1.setPosition(color);}
        //else if (led == LED.LED2) {LED2.setPosition(color);}
        //else if (led == LED.LED3) {LED3.setPosition(color);}
    }

    public double getLedColor(LED led) {
        if (led == LED.CHAINED || led == LED.LED1) {return LED1.getPosition();}
        //else if (led == LED.LED2) {return LED2.getPosition();}
        //else if (led == LED.LED3) {return LED3.getPosition();}
        else return 1738;
    }

}
