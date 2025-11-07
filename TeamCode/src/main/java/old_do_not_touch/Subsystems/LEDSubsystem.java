package old_do_not_touch.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

public class LEDSubsystem {
    private Servo LED0,LED1, LED2;
    private LED ledI;

    public LEDSubsystem(HardwareMap map) {
        LED0 = map.get(Servo.class, "led0");
        LED1 = map.get(Servo.class, "led1");
        LED2 = map.get(Servo.class, "led2");
        ledI = map.get(LED.class, "led");
    }

    public void setLedColor(int index, double color) {
        if (index == 0) {LED0.setPosition(color);}
        else if (index == 1) {LED1.setPosition(color);}
        else if (index == 2) {LED2.setPosition(color);}
        else if (index == 3) {LED0.setPosition(color); LED1.setPosition(color); LED2.setPosition(color);}
    }

    public double getLedColor(int index) {
        double out;
        if (index == 0) {out = LED0.getPosition();}
        else if (index == 1) {out = LED1.getPosition();}
        else if (index == 2) {out = LED2.getPosition();}
        else { out =  0000.0000;}
        return out;
    }


}
