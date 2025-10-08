package org.firstinspires.ftc.teamcode.color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.color.LED;

public class ColorDriver {
    private boolean chained;
    private Servo LED1, LED2, LED3;

    public ColorDriver(HardwareMap map, boolean chainedV) {
        this.chained = chainedV;

        this.LED1 = map.get(Servo.class, "led1");
        if (!chainedV) { this.LED2 = map.get(Servo.class, "led2"); this.LED3 = map.get(Servo.class, "led3");}
    }

    public void setLedColor(LED led, double color) {
        if (led == LED.CHAINED || led == LED.LED1) {LED1.setPosition(color);}
        else if (led == LED.LED2) {LED2.setPosition(color);}
        else if (led == LED.LED3) {LED3.setPosition(color);}
    }

    public double getLedColor(LED led) {
        if (led == LED.CHAINED || led == LED.LED1) {return LED1.getPosition();}
        else if (led == LED.LED2) {return LED2.getPosition();}
        else if (led == LED.LED3) {return LED3.getPosition();}
        else return 1738;
    }

}
