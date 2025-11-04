package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.CarouselPosition;
import org.firstinspires.ftc.teamcode.Constants.ColorConstants;

public class CarouselSubsystem extends SubsystemBase {
    public static class Ball {
        public int position;
        public ColorConstants.BallColor color;
        public boolean full;

        public Ball(int pos, ColorConstants.BallColor colorI) {
            this.position = pos;
            this.color = colorI;
            this.full = false;
        }
        public Ball(int pos, ColorConstants.BallColor colorI, boolean fullI) {
            this.position = pos;
            this.color = colorI;
            this.full = fullI;
        }

        public void setPosition(int p) {
            position = p;
        }
        public void setColor(ColorConstants.BallColor c) {
            this.color = c;
        }
    }
    private Servo carousel;
    private RevColorSensorV3 sensor;
    private LEDSubsystem led;
    private int counter;
    public boolean inOne, inTwo, inThree, testBool;
    public Ball ball1, ball2, ball3;
    public double distance;
    private int flush132C = 0;

    /// Initialize Carousel Components with Constructors
    public CarouselSubsystem(HardwareMap map) {
        sensor = map.get(RevColorSensorV3.class, "HopperColorSensor");
        carousel = map.get(Servo.class, "CarouselServo");
        led = new LEDSubsystem(map);

        carousel.setDirection(Servo.Direction.REVERSE);

        this.ball1 = new Ball(1, ColorConstants.BallColor.NULL);
        this.ball2 = new Ball(2, ColorConstants.BallColor.NULL);
        this.ball3 = new Ball(3, ColorConstants.BallColor.NULL);

        this.counter = 0;
    }

    // Constructor to start the carousel at certain position
    public CarouselSubsystem(HardwareMap map, int startingPos) {
        sensor = map.get(RevColorSensorV3.class, "HopperColorSensor");
        carousel = map.get(Servo.class, "CarouselServo");
        led = new LEDSubsystem(map);

        carousel.setDirection(Servo.Direction.REVERSE);

        this.ball1 = new Ball(1, ColorConstants.BallColor.NULL);
        this.ball2 = new Ball(2, ColorConstants.BallColor.NULL);
        this.ball3 = new Ball(3, ColorConstants.BallColor.NULL);

        this.counter = startingPos;
    }

    // Functions to be run every loop.
    public void periodic() {
        //this.toPos(CarouselPosition.servoPosition(this.getCounter()));

        this.updateBallPositions();
        //this.updateLEDColors();
        this.limitCounter();
        this.distance = this.getDistance();
    }

    public boolean allOccupied() {
        return (this.inOne && this.inTwo && this.inThree);
    }

    public void flush132() {
        if (flush132C < 10) {
            flush132C += 1;
            carousel.setPosition(getPosDouble() + 0.004);
        }
        else {
            carousel.setPosition(getPosDouble() - 0.01);
            flush132C = 0;
        }
    }

    /// Returns Raw Position of Carousel Servo
    public double getPosDouble() {
        return carousel.getPosition();
    }

    public void toPos14() {
        carousel.setPosition(getPosDouble() + (0.2/3.0 * 3.0/4.0));
    }
    public void toPos34() {
        carousel.setPosition(getPosDouble() + (0.2/3.0 * 1.0/4.0));
    }

    public void intakeBall() {
        carousel.setPosition(carousel.getPosition() - CarouselPosition.test);
        toPos(CarouselPosition.servoPosition(counter) + CarouselPosition.test);
    }

    /// Returns which 'capsule' is at the Intake hole
    public int getPosInt() {
        int counterNew = counter + 3;

        if (counterNew % 3 == 0) {
            return 1;
        }
        else if (counterNew % 3 == 1) {
            return 2;
        }
        else if (counterNew % 3 == 2) {
            return 3;
        }
        else return 0;
    }

    /// Returns if specified 'capsule' at Intake hole
    public boolean atPosInt(int i) {
        return getPosInt() == i;
    }

    //TODO: updateBallColors() run only when distance sensor past threshold
    public void updateBallColors() {
        if (atPosInt(1)) {
            this.ball3.setColor(ColorConstants.detectColor(colorSensorRed(), colorSensorGreen(), colorSensorBlue()));
            this.ball3.setPosition(1);
        }
        else if (atPosInt(2)) {
            this.ball1.setColor(ColorConstants.detectColor(colorSensorRed(), colorSensorGreen(), colorSensorBlue()));
            this.ball1.setPosition(2);
        }
        else if (atPosInt(3)) {
            this.ball2.setColor(ColorConstants.detectColor(colorSensorRed(), colorSensorGreen(), colorSensorBlue()));
            this.ball2.setPosition(3);
        }

    }
    public void updateBallPositions() {
        if (atPosInt(1)) {
           inOne = (this.getDistance() <= CarouselPosition.distanceMax);
        }
        else if (atPosInt(2)) {
            inTwo = (this.getDistance() <= CarouselPosition.distanceMax);
        }
        else if (atPosInt(3)) {
            inThree = (this.getDistance() <= CarouselPosition.distanceMax);
        }
    }
    public void updateLEDColors() {
        double b1c, b2c, b3c;

        if (ball1.color == ColorConstants.BallColor.GREEN) {
            b1c = ColorConstants.BALL_GREEN;
        }
        else if (ball1.color == ColorConstants.BallColor.PURPLE) {
            b1c = ColorConstants.BALL_PURPLE;
        }
        else b1c = ColorConstants.RED;

        if (ball2.color == ColorConstants.BallColor.GREEN) {
            b2c = ColorConstants.BALL_GREEN;
        }
        else if (ball2.color == ColorConstants.BallColor.PURPLE) {
            b2c = ColorConstants.BALL_PURPLE;
        }
        else b2c = ColorConstants.RED;

        if (ball3.color == ColorConstants.BallColor.GREEN) {
            b3c = ColorConstants.BALL_GREEN;
        }
        else if (ball3.color == ColorConstants.BallColor.PURPLE) {
            b3c = ColorConstants.BALL_PURPLE;
        }
        else b3c = ColorConstants.RED;

        led.setLedColor(0, b1c);
        led.setLedColor(1, b2c);
        led.setLedColor(2, b3c);
    }

    /// Sets Position Counter
    public void setCounter(int n) {
        this.counter = n;
    }
    public int getCounter() {
        return this.counter;
    }

    /// Keeps the Servo within bounds (run in main loop)
    public void limitCounter() {
        if (this.counter < CarouselPosition.counterMin) {this.counter = CarouselPosition.counterMin;}
        else if (this.counter > CarouselPosition.counterMax) {this.counter = CarouselPosition.counterMax;}
    }

    /// Sets position of carousel servo
    public void toPos(double p) {
        carousel.setPosition(p);
    }

    /// counter
    public void toPosCounter() {
        carousel.setPosition(CarouselPosition.servoPosition(counter));
    }

    /// Returns distance from Color Sensor
    public double getDistance() {
        return sensor.getDistance(DistanceUnit.MM);
    }

    /// Color Sensor Returns
    public ColorConstants.RGB colorSensorReturn() {
        return new ColorConstants.RGB(sensor.red(),sensor.green(),sensor.blue());
    }
    public int colorSensorRed() {
        return sensor.red();
    }
    public int colorSensorGreen() {
        return sensor.green();
    }
    public int colorSensorBlue() {
        return sensor.blue();
    }


}
