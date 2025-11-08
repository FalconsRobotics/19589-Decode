package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HopperSubsystem extends SubsystemBase {
    /// Object that references the center servo on the hopper.
    private final CRServo servo;

    /// Object that references the input wire of the Axon Servo
    private final AnalogInput servoEncoder;

    private final double position1 = 1.1;
    private final double position2 = 2.2;
    private final double position3 = 3.3;

    /**
     * Initialize the HopperSubsystem and initializes device settings
     * @param map Uses the HardwareMap from your Auto/TeleOp to intiailize all of the hardware.
     */
    public HopperSubsystem(HardwareMap map) {
        servo = map.get(CRServo.class, "centerServo");
        servoEncoder = map.get(AnalogInput.class, "servoEncoder");

        servo.setDirection(CRServo.Direction.FORWARD);
    }


    ///@return The position of the Axon Servo with range of 0.0 - 3.3 volts
    public double getServoPosition() {
        return servoEncoder.getVoltage();
    }

    /**
     * @param unit The unit of the angle you are returning (Rad or Deg)
     * @return The angle of the Center Servo in either Degrees or Radians
     */
    public double getServoAngle(AngleUnit unit) {
        double degrees = getServoPosition() * (360 / servoEncoder.getMaxVoltage());;

        if (unit == AngleUnit.DEGREES) {return degrees;}
        else {return Math.toRadians(degrees);}
    }


    ///@return The power of the rotating servo
    public double getServoPower() {
        return servo.getPower();
    }

}
