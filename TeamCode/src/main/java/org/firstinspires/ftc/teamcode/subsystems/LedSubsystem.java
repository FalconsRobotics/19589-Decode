package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

    public final Servo[] LED = new Servo[3];
    public final RevBlinkinLedDriver strip;

    /**
     * Constructor for all of the LED's on the robot
     * @param map HardwareMap object from OpMode
     */
    public LedSubsystem(HardwareMap map) {
        LED[0] = map.get(Servo.class, "led0");
        LED[1] = map.get(Servo.class, "led1");
        LED[2] = map.get(Servo.class, "led2");

        strip = map.get(RevBlinkinLedDriver.class, "ledStrip");
    }

    /**
     * Sets the color of an LED
     * @param index Which led to control
     * @param color Input color value
     */
    public void setColor(int index, double color) {
        if (index > 2 || index < 0) {
            LED[0].setPosition(color);
            LED[1].setPosition(color);
            LED[2].setPosition(color);
        }
        else {
            LED[index].setPosition(color);
        }
    }

    public void setColor(Object index, double color) {
        if (index.getClass() == int.class) {

        }
    }
}
