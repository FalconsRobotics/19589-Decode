package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.CarouselConstants;


public class Carousel {
    private CRServo carousel;
    private Servo newCarousel;
    private RevColorSensorV3 sensor;
    private AnalogInput pos;
    public double counter = 0;

    public class pidOut {
        double error;
        double target;
        double power;
        public pidOut(double errorI, double targetI, double powerI) {
            this.error = errorI;
            this.target = targetI;
            this.power = powerI;
        }
    }


    /// Return if ball in certain position in carousel
    public boolean inOne, inTwo, inThree;

    /// Initialize Carousel Components
    public Carousel(HardwareMap map) {
        carousel = map.get(CRServo.class, "carousel");
        newCarousel = map.get(Servo.class, "new");
        sensor = map.get(RevColorSensorV3.class, "distance-sensor");
        carousel.setDirection(CRServo.Direction.FORWARD);
        pos = map.get(AnalogInput.class, "carousel-pos");
    }
    public Carousel(HardwareMap map, boolean inOneI, boolean inTwoI, boolean inThreeI) {
        carousel = map.get(CRServo.class, "carousel");
        sensor = map.get(RevColorSensorV3.class, "distance-sensor");
        carousel.setDirection(CRServo.Direction.REVERSE);
        pos = map.get(AnalogInput.class, "carousel-pos");
        this.inOne = inOneI; this.inTwo = inTwoI; this.inThree = inThreeI;
    }
    double lastPos = 0;
    /// Returns Position of Carousel Servo
    public double getPos() {
        return pos.getVoltage();
    }

    public boolean getInOne() {
        return inOne;
    }
    public boolean getInTwo() {
        return inTwo;
    }
    public boolean getInThree() {
        return inThree;
    }
    public void setInOne(boolean i) {
        this.inOne = i;
    }
    public void setInTwo(boolean i) {
        this.inTwo = i;
    }
    public void setInThree(boolean i) {
        this.inThree = i;
    }

    public boolean inRange(double input, double target, double margin) {
        if (input > target - margin && input < target + margin) {
            return true;
        }
        else return false;
    }

    /// Sets power of Carousel Servo
    public void setPower(double power) {
        carousel.setPower(power);
    }

    /// Returns Power of Carousel Servo
    public double getPower() {
        return carousel.getPower();
    }

    /// Returns distance from Color Sensor
    public double getDistance() {
        return sensor.getDistance(DistanceUnit.MM);
    }

    /// Returns if Carousel is near set position (0 if not)
    public int getPosInt() {
        if (inRange(getPos(), CarouselConstants.input1, 0.1)) {
            return 1;
        }
        else if (inRange(getPos(), CarouselConstants.input2, 0.1)) {
            return 2;
        }
        else if (inRange(getPos(), CarouselConstants.input3, 0.1)) {
            return 3;
        }
        else return 0;
    }

    /// Sets power of Carousel Servo until set Position
    public double intakeToPos(int targetPos) {
        double currentPos = getPos();
        double error;
        double FF = 0.2;
        if (targetPos == 1) {error = CarouselConstants.input1 - currentPos;}
        else if (targetPos == 2) {error = CarouselConstants.input2 - currentPos;}
        else if (targetPos == 3) {error = CarouselConstants.input3 - currentPos;}
        else return 0;

        double errorAbs = Math.abs(error);

        return Math.min(FF, errorAbs);

    }
    public double intakeToPos(double target) {
        double error = target - getPos();
        double power = Math.abs(CarouselConstants.P_VALUE * error);

        carousel.setPower(power);
        return error;
    }

    public double errorReturn(double target) {
        return target - getPos();
    }

    public double toPosReturn(double p) {
        newCarousel.setPosition(p);
        return newCarousel.getPosition();
    }

    public boolean between01() {
        return getPos() > 0 && getPos() < CarouselConstants.input1 ? true : false;
    }
    public boolean between12() {
        return getPos() > CarouselConstants.input1 && getPos() < CarouselConstants.input2 ? true : false;
    }
    public boolean between23() {
        return getPos() > CarouselConstants.input2 && getPos() < CarouselConstants.input3 ? true : false;
    }
}
