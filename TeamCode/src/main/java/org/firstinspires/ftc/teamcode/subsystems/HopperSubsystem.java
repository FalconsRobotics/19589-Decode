package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HopperSubsystem extends SubsystemBase {
    /// Object that references the center servo on the hopper.
    private final CRServo servo;

    /// Object that references the axle's external encoder
    private final DcMotor encoder;

    /// Object that references the Axon Servo's encoder wire
    //private final AnalogInput axonEncoder;

    /**
     * Initialize the HopperSubsystem and initializes device settings
     * @param map Uses the HardwareMap from your Auto/TeleOp to intiailize all of the hardware.
     */
    public HopperSubsystem(HardwareMap map) {
        servo = map.get(CRServo.class, "CarouselServo");
        encoder = map.get(DcMotor.class, "motorEncoder");
        //axonEncoder = map.get(AnalogInput.class, "axonEncoder");

        servo.setDirection(CRServo.Direction.FORWARD);
        encoder.setDirection(DcMotor.Direction.FORWARD);
    }

    /// @return The position of the Axon Servo ranging from 0.0-3.3v
    /*public double getAxonEncoder() {
        return axonEncoder.getVoltage();
    }*/

    ///@return The position of the servo using the Through Bore Encoder
    public int getServoPosition() {
        return encoder.getCurrentPosition();
    }

    ///@return The power of the rotating servo
    public double getServoPower() {
        return servo.getPower();
    }

    /**
     * Sets power of the center servo
     * @param p power value
     */
    public void setServoPower(double p) {
        servo.setPower(p);
    }

    /**
     * Rotates servo to an encoder position
     * @param target target position
     */
    public void toPosition(int target) {
        int error = target - getServoPosition();
        double MAX = 0.1;
        double P_GAIN = 0.005;

        double prop_output = error * P_GAIN;
        double final_power = Math.max(-MAX, Math.min(MAX, prop_output));

        setServoPower(final_power);
    }

    /**
     * @return The angle of the axon servo
     * @param unit Unit of return angle
     */
    /*
    public double getAxonAngle(AngleUnit unit) {
        double degrees = getAxonEncoder() * (360 / axonEncoder.getMaxVoltage());

        return (unit == AngleUnit.DEGREES ? degrees : Math.toRadians(degrees));
    }*/
}
