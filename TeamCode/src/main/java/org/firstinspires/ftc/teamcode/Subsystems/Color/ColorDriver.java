package org.firstinspires.ftc.teamcode.Subsystems.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ColorDriver {
    private Servo LED0,LED1, LED2;

    public ColorDriver(HardwareMap map) {
        LED0 = map.get(Servo.class, "led0");
        LED1 = map.get(Servo.class, "led1");
        LED2 = map.get(Servo.class, "led2");
    }

    public void setLedColor(LED led, double color) {
        if (led == LED.CHAINED || led == LED.LED0) {LED0.setPosition(color);}
        else if (led == LED.LED1) {LED1.setPosition(color);}
        else if (led == LED.LED2) {LED2.setPosition(color);}
        else if (led == LED.ALL) {LED0.setPosition(color); LED1.setPosition(color); LED2.setPosition(color);}
    }

    public double getLedColor(LED led) {
        if (led == LED.CHAINED || led == LED.LED0) {return LED0.getPosition();}
        else if (led == LED.LED1) {return LED1.getPosition();}
        else if (led == LED.LED2) {return LED2.getPosition();}
        else return 1738;
    }

    public void setLED0(double color) {LED0.setPosition(color);}
    public void setLED1(double color) {LED1.setPosition(color);}
    public void setLED2(double color) {LED2.setPosition(color);}
    public void setLEDChained(double color) {LED0.setPosition(color);}
    public void setLEDAll(double color) {LED0.setPosition(color); LED1.setPosition(color); LED2.setPosition(color);}

    public double getLed0() {
        return LED0.getPosition();
    }
    public double getLed1() {
        return LED1.getPosition();
    }
    public double getLed2() {
        return LED2.getPosition();
    }

}
