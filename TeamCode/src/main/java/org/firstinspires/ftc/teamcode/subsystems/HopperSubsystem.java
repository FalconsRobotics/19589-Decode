package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.Color;

public class HopperSubsystem extends SubsystemBase {
    /// Object that references the center servo on the hopper.
    private final CRServo servo;

    /// Object that references the axle's external encoder
    private final DcMotor encoder;

    /// Object that references the Axon Servo's encoder wire
    private final RevColorSensorV3 topSensor, bottomSensor;


    /**
     * Initialize the HopperSubsystem and initializes device settings
     * @param map Uses the HardwareMap from your Auto/TeleOp to intiailize all of the hardware.
     */
    public HopperSubsystem(HardwareMap map) {
        servo = map.get(CRServo.class, "CarouselServo");
        encoder = map.get(DcMotor.class, "MotorEncoder");

        topSensor = map.get(RevColorSensorV3.class, "TopSensor");
        bottomSensor = map.get(RevColorSensorV3.class, "BottomSensor");

        servo.setDirection(CRServo.Direction.REVERSE);
        encoder.setDirection(DcMotor.Direction.FORWARD);
    }

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

        this.setServoPower(final_power);
    }

    /// @return The distance from the Top Sensor
    public double getTopDistance() {
        return topSensor.getDistance(DistanceUnit.MM);
    }

    /// @return The distance from the Bottom Sensor
    public double getBottomDistance() {
        return bottomSensor.getDistance(DistanceUnit.MM);
    }

    /// @return The color returned from the Top Sensor
    public NormalizedRGBA getTopColor() {
        return topSensor.getNormalizedColors();
    }

    /// @return The color returned from the Bottom Sensor
    public NormalizedRGBA getBottomColor() {
        return bottomSensor.getNormalizedColors();
    }

    public void method() {

    }
}