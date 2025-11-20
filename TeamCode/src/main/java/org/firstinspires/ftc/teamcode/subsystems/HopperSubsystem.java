package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.HopperConstants;

import java.util.List;


public class HopperSubsystem extends SubsystemBase {
    private CRServo carouselServo;
    private DcMotor motorEncoder;
    private DigitalChannel magnetSwitch;
    private ColorRangeSensor colorSensor;

    public List<Integer> colors;

    public double hopperPosition = 0.0;

    public double hopperEncoderTicks = 0.0;
    public double hopperEncoderTicksTarget = 0.0;

    public double hopperEncoderTicksError = 0.0;
    public double lastHopperEncoderTicksError;

    public double hopperPTerm = 0.0;
    public double hopperITerm = 0.0;
    public double hopperDTerm = 0.0;
    public double hopperFTerm = 0.0;

    public double hopperServoPower = 0.0;

    private boolean hopperMotorIsBusy = false;

    public HopperSubsystem(HardwareMap map) {
        carouselServo = map.get(CRServo.class, "CarouselServo");
        motorEncoder = map.get(DcMotor.class, "MotorEncoder");
        magnetSwitch = map.get(DigitalChannel.class, "MagnetSwitch");
        colorSensor = map.get(ColorRangeSensor.class, "ColorSensor");

        carouselServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        readHopperPosition();
        rotateHopperOnePosition(1);
    }

    public void findHopperHomePosition() {
        while (magnetSwitch.getState()) {
            carouselServo.setPower(0.15);
        }

        carouselServo.setPower(0);

        motorEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetPosition(int direction) {
        this.hopperEncoderTicksTarget += Math.signum(direction) * 2731;
        this.hopperEncoderTicks = Math.round(Math.round(this.hopperEncoderTicks / (8192.0 / 3.0)) * (8192.0 / 3.0));
    }

    public void rotateHopperOnePosition(int direction) {
        this.hopperEncoderTicksError = this.hopperEncoderTicksTarget - this.hopperEncoderTicks;

        this.hopperPTerm = this.hopperEncoderTicksError * HopperConstants.kp;
        this.hopperDTerm = (this.hopperEncoderTicksError - this.lastHopperEncoderTicksError) * HopperConstants.kd;

        this.lastHopperEncoderTicksError = this.hopperEncoderTicksError;

        if (Math.abs(this.hopperEncoderTicksError) > 125) {
            this.hopperFTerm = Math.signum(direction) * HopperConstants.kf;
        } else {
            this.hopperFTerm = 0.0;
        }

        this.hopperServoPower = (this.hopperPTerm + this.hopperDTerm) * HopperConstants.hopperSpeedFactor + this.hopperFTerm;

        if (Math.abs(this.hopperServoPower) > HopperConstants.hopperSpeedFactor) {
            this.hopperServoPower = (Math.abs(this.hopperServoPower) / this.hopperServoPower) * HopperConstants.hopperSpeedFactor;
        }

        if (Math.abs(this.hopperEncoderTicksError) > 45) {
            carouselServo.setPower(this.hopperServoPower);
        } else {
            carouselServo.setPower(0);
        }
    }

    public void readHopperPosition() {
        this.hopperEncoderTicks = motorEncoder.getCurrentPosition();
        this.hopperPosition = Math.round(this.hopperEncoderTicks % 3 / (8192.0 / 3.0));
    }

    /**
     *
     */
    public void readIntakePositionColor() {
        int redColorChannel;
        int greenColorChannel;
        int blueColorChannel;

        if (colorSensor.getDistance(DistanceUnit.CM) <= 5) {
            redColorChannel = colorSensor.red();
            greenColorChannel = colorSensor.green();
            blueColorChannel = colorSensor.blue();

            if (redColorChannel > greenColorChannel) {
                colors.set((int) this.hopperPosition, 0);
            }
        }
    }
}