package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.CarouselPosition;
import org.firstinspires.ftc.teamcode.Constants.ColorConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ControllerSubsystem;

import java.util.List;

@TeleOp(name = "TeleOp - Open Warfare", group = "Main")
public class Main extends OpMode {
    // Right now, we're not going to use SubsystemCollection, the singleton
    // that holds all of our subsystem classes, until we start using SolversLib.
    // SubsystemCollection sys;

    // Objects for each of our subsystems
    MecanumDriveBase drivebase;
    IntakeElevatorSubsystem intake;
    CarouselSubsystem hopper;
    ShooterSubsystem shooter;
    LEDSubsystem led;

    // Here, we decided to use SolversLib's GamepadEx controller for its
    // "whenJustPressed()" and "isDown()" functionality to maintain simplicity.
    GamepadEx Gamepad1;
    GamepadEx Gamepad2;
    ControllerSubsystem.Toggle autoMode, zeroPos;

    LimelightSubsystem ll;

    // Distance sensor variable
    public boolean sensorUpdated = false;
    public double teamColor = ColorConstants.GREEN;
    ColorConstants.RGB rgb = new ColorConstants.RGB(0,0,0);
    ColorConstants.BallColor ballColor = ColorConstants.BallColor.NULL;
    ColorConstants.Pattern.CyclicGradient cycle= new ColorConstants.Pattern.CyclicGradient(ColorConstants.RED);

    @Override public void init() {
        // Set up the various subsystems.
        // TODO: When adding commands, add in SubsystemsCollection
        drivebase = new MecanumDriveBase(hardwareMap);
        intake = new IntakeElevatorSubsystem(hardwareMap);
        hopper = new CarouselSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        led = new LEDSubsystem(hardwareMap);

        ll = new LimelightSubsystem(hardwareMap);

        // Initialize the Gamepads for use from SolversLib.
        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);
        autoMode = new ControllerSubsystem.Toggle(false);
        zeroPos = new ControllerSubsystem.Toggle(false);

        // Enable bulk cached reading to speed up I2C read times.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        shooter.setPower(ShooterConstants.FIRING_SPEED_SHORT);
        shooter.farSpeed = false;
    }

    @Override public void init_loop() {
        Gamepad2.readButtons();
        hopper.setCounter(1);
        hopper.toPos(CarouselPosition.servoPosition(hopper.getCounter()));

        if (Gamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            teamColor = ColorConstants.RED;
        }
        else if (Gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            teamColor = ColorConstants.BLUE;
        }
    }

    @Override public void loop() {
        // Call the periodic functions from the SubsystemCollection
        // class. Functions that need to be run every loop.
        drivebase.periodic();
        hopper.periodic();
        shooter.periodic(gamepad2);

        led.setLedColor(0, cycle.getNextPosition(ColorConstants.GREEN, ColorConstants.SAGE));
        if (shooter.farSpeed) {
            led.setLedColor(1, ColorConstants.ORANGE);
        }
        else {
            led.setLedColor(1, ColorConstants.VIOLET);
        }

        led.setLedColor(0, teamColor);
        led.setLedColor(2, teamColor);


        // Read out Color Sensor values
        rgb.setRGB(hopper.colorSensorRed(), hopper.colorSensorGreen(), hopper.colorSensorBlue());

        // SolversLib requires that we read the gamepad every loop.
        Gamepad1.readButtons();
        Gamepad2.readButtons();

        // Switch between automated carousel rotation
        if (Gamepad2.wasJustPressed(GamepadKeys.Button.START)) {autoMode.toggle();}
        //if(Gamepad2.wasJustPressed(GamepadKeys.Button.BACK)) {zeroPos.toggle();}

        // Subsystem control
        shooter.setPower(shooter.farSpeed ? ShooterConstants.FIRING_SPEED_LONG : ShooterConstants.FIRING_SPEED_SHORT);
        if (Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {  shooter.farSpeed = false;  }
        if (Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { shooter.farSpeed = true; }

        //Shutoff intake when carousel is full
        if (Gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5) {
            intake.setPower(-1);
        } else {
            intake.setPower(1);
        }



        // Gather joystick values from LX, LY (inverted, so up is forward y),
        // and RX (for turning) on Gamepad1.
        double cX = Gamepad1.getLeftX();
        double cY = Gamepad1.getLeftY();
        double cRX = Gamepad1.getRightX();
        double cRY = -Gamepad1.getRightY();

        double turnAngle = Math.atan2(cRX, cRY);
        double speedMultiplier = 1 - (Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.7);

        drivebase.Drive(cX * speedMultiplier, cY * speedMultiplier, cRX * speedMultiplier, -1);

//        drivebase.DriveFieldCentricWithLock(cX * speedMultiplier, cY * speedMultiplier, cRX, cRY);

        // Controls for the hopper. LB cycles it left, RB cycles it right a single time.
        // D-Pad down returns it back to zero / the center;
        if (Gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            hopper.setCounter(hopper.getCounter() - 1);
            hopper.toPos(hopper.getPosDouble() - 0.07);
        }
        if (Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            hopper.setCounter(0);
            hopper.toPosCounter();
        }
        if (Gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            hopper.flush132();
        }
        if (Gamepad2.wasJustPressed(GamepadKeys.Button.BACK)) {
            hopper.toPos(CarouselPosition.zeroPos);
        }

        /// Intaking
        if (Gamepad2.wasJustPressed((GamepadKeys.Button.RIGHT_BUMPER))) {
            hopper.setCounter(hopper.getCounter() + 1);
            hopper.toPos(CarouselPosition.servoPosition(hopper.getCounter()));
        }
        /*if (Gamepad2.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            hopper.setCounter(hopper.getCounter() + 1);
            hopper.toPos(CarouselPosition.servoPosition(hopper.getCounter()));
        }*/



        telemetry.addLine("---------- COACH ----------");
        telemetry.addLine();
        telemetry.addData("Shooting Position", (shooter.farSpeed ? "FAR" : "CLOSE"));
        telemetry.addLine();
        telemetry.addData("Shooter Velocity", shooter.getVelocity());
        telemetry.addLine();

        if (teamColor == ColorConstants.RED) {
            telemetry.addData("Angle to RED", ll.angleToGoalRed());
        } else if (teamColor == ColorConstants.BLUE) {
            telemetry.addData("Angle to BLUE", ll.angleToGoalBlue());
        }

        telemetry.addLine("--- LOOK UP HERE TEIGAN ---");

        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();

        telemetry.addLine("------------------------------");
        telemetry.addData("CPos", CarouselPosition.servoPosition(hopper.getCounter()));
        telemetry.addLine("------------------------------");
        telemetry.addData("Counter", hopper.getCounter());
        telemetry.addData("Servo Position", hopper.getPosDouble());
        telemetry.addLine("------------------------------");
        telemetry.addData("Heading", drivebase.odo.getHeading(AngleUnit.DEGREES));
        telemetry.addLine("------------------------------");
        telemetry.addData("Automated Carousel", autoMode.isTrue());
        telemetry.addData("Zero Pos On", zeroPos.isTrue());
        telemetry.addData("Distance", hopper.distance);
        telemetry.update();
    }
}