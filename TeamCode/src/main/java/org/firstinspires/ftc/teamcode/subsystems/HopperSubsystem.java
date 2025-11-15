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
    private final RevColorSensorV3 bottomSensor;

    /// Object that references the magnet switch on rotor
    private final DigitalChannel magSwitch;

    /// Booleans used during initialization of the hopper
    public boolean isHomed, magnetSeenDuringHoming;

    /// Which position rotor is at (null if in between)
    public Integer currentPosition;

    ///
    public int targetPosition;


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

        bottomSensor = map.get(RevColorSensorV3.class, "BottomSensor");

        isHomed = false;
        magnetSeenDuringHoming = false;

        currentPosition = null;
        targetPosition = 0;
    }

    /// Run During init() Stage of TeleOp; Rotates hopper until it is physically zeroed with the magnet switch
    public void runToMagnetZero() {
        if (!this.isHomed) {
            if (!this.magnetIsActive()) {
                this.setServoPower(HopperConstants.HOMING_POWER);
            }
            else {
                this.setServoPower(0.0);
                this.isHomed = true;
            }
        }
        else {
            this.setServoPower(0.0);
            this.zeroEncoder();
            this.targetPosition = 0;
        }
    }

    ///@return The position of the servo using the Through Bore Encoder
    public int getEncoderPosition() {
        return encoder.getCurrentPosition();
    }

    /// @return The current position of the hopper
    public Integer getHopperPosition() {
        if (getEncoderPosition() <= 1 * (HopperConstants.TICKS_PER_STEP) + HopperConstants.TOLERANCE_TICKS
                && getEncoderPosition() >= 1 * (HopperConstants.TICKS_PER_STEP - HopperConstants.TOLERANCE_TICKS)) {
            return 1;
        }
        else if (getEncoderPosition() <= 2 * (HopperConstants.TICKS_PER_STEP) + HopperConstants.TOLERANCE_TICKS
                && getEncoderPosition() >= 2 * (HopperConstants.TICKS_PER_STEP - HopperConstants.TOLERANCE_TICKS)) {
            return 2;
        }
        else if (
                (getEncoderPosition() <= 3 * (HopperConstants.TICKS_PER_STEP) + HopperConstants.TOLERANCE_TICKS
                        && getEncoderPosition() >= 3 * (HopperConstants.TICKS_PER_STEP - HopperConstants.TOLERANCE_TICKS))
                ||
                        (getEncoderPosition() <= HopperConstants.TOLERANCE_TICKS)
                                && getEncoderPosition() >= -HopperConstants.TOLERANCE_TICKS) {
            return 2;
        }
        else return null;
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
    public void toPositionRaw(int target) {
        int error = target - getEncoderPosition();
        double proportional = (double) target * HopperConstants.MOVE_KP;
        double directionalMoveMax = Math.signum(error) * HopperConstants.MOVE_MAX;

        if (error <= HopperConstants.TOLERANCE_TICKS) {
            setServoPower(0);
        }
        else {
            if (Math.signum(error) == 1) {
                setServoPower(Math.min(directionalMoveMax, proportional));
            }
            else if (Math.signum(error) == -1) {
                setServoPower(Math.max(directionalMoveMax, proportional));
            }
            else {
                setServoPower(0);
            }
        }
    }

    /**
     * Rotates servo counter-clockwise to 1 of 3 hopper positions
     * @param position target hopper position
     */
    public void intakeToHopperPosition(Integer position) {

    }

    /**
     * Rotates servo clockwise to 1 of 3 hopper positions
     * @param position target hopper position
     */
    public void extakeToHopperPosition(Integer position) {

    }

    /// Moves ccw 1 position
    public void intakeOneTick() {
        toPositionRaw(getEncoderPosition() + (int) HopperConstants.TICKS_PER_STEP);
    }

    /// Moves cw 1 position
    public void extakeOneTick() {
        toPositionRaw(getEncoderPosition() - (int) HopperConstants.TICKS_PER_STEP);
    }

    /// @return The distance from the Top Sensor
//    public double getTopDistance() {
//        return topSensor.getDistance(DistanceUnit.MM);
//    }

    /// @return The distance from the Bottom Sensor
    public double getBottomDistance() {
        return bottomSensor.getDistance(DistanceUnit.MM);
    }

    /// @return The color returned from the Top Sensor
//    public NormalizedRGBA getTopColor() {
//        return topSensor.getNormalizedColors();
//    }

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