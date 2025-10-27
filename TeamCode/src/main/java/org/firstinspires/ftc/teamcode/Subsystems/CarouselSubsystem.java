package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.CarouselPosition;
import org.firstinspires.ftc.teamcode.Subsystems.Color.Color;
import org.firstinspires.ftc.teamcode.Subsystems.Color.ColorDriver;

public class CarouselSubsystem {
    private Servo carousel;
    private RevColorSensorV3 sensor;
    public ColorDriver led;
    private AnalogInput pos;
    private int counter;
    public boolean inOne, inTwo, inThree;
    public static class ColorOutput {
        public double RED, GREEN, BLUE;
        public ColorOutput(double redIn, double greenIn, double blueIn) {
            this.RED = redIn;
            this.GREEN = greenIn;
            this.BLUE = blueIn;
        }

        public double getRed() {
            return this.RED;
        }
        public double getGreen() {
            return this.GREEN;
        }
        public double getBlue() {
            return this.BLUE;
        }

    }

    public static class Ball {
        public int position;
        public Color.BallColor color;

        public Ball(int pos, Color.BallColor colori) {
            position = pos;
            color = colori;
        }

        public void setPosition(int p) {
            position = p;
        }
        public void setColor(Color.BallColor c) {
            this.color = c;
        }
    }

    public Ball ball1, ball2, ball3;

    /// Initialize Carousel Components with Constructors
    public CarouselSubsystem(HardwareMap map) {
        sensor = map.get(RevColorSensorV3.class, "distance-sensor");
        pos = map.get(AnalogInput.class, "carousel-pos");
        carousel = map.get(Servo.class, "new");
        led = new ColorDriver(map);

        carousel.setDirection(Servo.Direction.REVERSE);

        this.ball1 = new Ball(1, Color.BallColor.NULL);
        this.ball2 = new Ball(2, Color.BallColor.NULL);
        this.ball3 = new Ball(3, Color.BallColor.NULL);

        this.counter = 0;
    }

    // Functions to be run every loop.
    public void periodic() {
        this.toPos(CarouselPosition.servoPosition(this.getCounter()));
        this.updateColors();
        this.updateBalls();
    }

    /// Returns Position of Carousel Servo
    public double getPosDouble() {
        return carousel.getPosition();
    }
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
    public boolean atPosInt(int i) {
        return getPosInt() == i;
    }
    public void updateBalls() {
        if (atPosInt(1)) {
            this.ball3.setColor(Color.detectColor(colorSensorRed(), colorSensorGreen(), colorSensorBlue()));
            this.ball3.setPosition(1);
        }
        else if (atPosInt(2)) {
            this.ball1.setColor(Color.detectColor(colorSensorRed(), colorSensorGreen(), colorSensorBlue()));
            this.ball1.setPosition(2);
        }
        else if (atPosInt(3)) {
            this.ball2.setColor(Color.detectColor(colorSensorRed(), colorSensorGreen(), colorSensorBlue()));
            this.ball2.setPosition(3);
        }

    }
    public void updateColors() {
        double b1c, b2c, b3c;

        if (ball1.color == Color.BallColor.GREEN) {
            b1c = Color.BALL_GREEN;
        }
        else if (ball1.color == Color.BallColor.PURPLE) {
            b1c = Color.BALL_PURPLE;
        }
        else b1c = Color.RED;

        if (ball2.color == Color.BallColor.GREEN) {
            b2c = Color.BALL_GREEN;
        }
        else if (ball2.color == Color.BallColor.PURPLE) {
            b2c = Color.BALL_PURPLE;
        }
        else b2c = Color.RED;

        if (ball3.color == Color.BallColor.GREEN) {
            b3c = Color.BALL_GREEN;
        }
        else if (ball3.color == Color.BallColor.PURPLE) {
            b3c = Color.BALL_PURPLE;
        }
        else b3c = Color.RED;

        led.setLED0(b1c);
        led.setLED1(b2c);
        led.setLED2(b3c);
    }

    /// LED
    public void updateLED() {

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

    /// Returns distance from Color Sensor
    public double getDistance() {
        return sensor.getDistance(DistanceUnit.MM);
    }

    /// Sets position of carousel servo
    public void toPos(double p) {
        carousel.setPosition(p);
    }

    /// Color Sensor Returns
    public ColorOutput colorSensorReturn() {
        return new ColorOutput(sensor.red(),sensor.green(),sensor.blue());
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
