package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.HopperConstants;


public class HopperSubsystem extends SubsystemBase {
    /// Object that references the center servo on the hopper.
    private final CRServo servo;

    /// Object that references the axle's external encoder
    private final DcMotor encoder;

    /// Object that references the Axon Servo's encoder wire
    private final RevColorSensorV3 topSensor, bottomSensor;

    /// Object that references the magnet switch on rotor
    private final DigitalChannel magSwitch;

    public boolean isHomed, magnetSeenDuringHoming;


    /**
     * Initialize the HopperSubsystem and initializes device settings
     * @param map Uses the HardwareMap from your Auto/TeleOp to intiailize all of the hardware.
     */
    public HopperSubsystem(HardwareMap map) {
        servo = map.get(CRServo.class, "CarouselServo");
        servo.setDirection(CRServo.Direction.REVERSE);

        encoder = map.get(DcMotor.class, "MotorEncoder");
        encoder.setDirection(DcMotor.Direction.FORWARD);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        magSwitch = map.get(DigitalChannel.class, "MagnetSwitch");
        magSwitch.setMode(DigitalChannel.Mode.INPUT);

        topSensor = map.get(RevColorSensorV3.class, "TopSensor");
        bottomSensor = map.get(RevColorSensorV3.class, "BottomSensor");

        isHomed = false;
        magnetSeenDuringHoming = false;
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
        double proportional = (double) target * HopperConstants.MOVE_KP;

        if (Math.signum(error) == 1) {
            setServoPower(Math.min(HopperConstants.MOVE_MIN, proportional));
        }
        else if (Math.signum(error) == -1) {
            setServoPower(Math.max(-HopperConstants.MOVE_MIN, proportional));
        }
        else {
            setServoPower(0);
        }
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

    /// @return Raw state of the magnet switch
    public boolean magnetStateRaw() {
        return magSwitch.getState();
    }

    /// @return If the magnet is making contact with switch
    public boolean magnetIsActive() {
        return !magnetStateRaw();
    }

    public void zeroEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}