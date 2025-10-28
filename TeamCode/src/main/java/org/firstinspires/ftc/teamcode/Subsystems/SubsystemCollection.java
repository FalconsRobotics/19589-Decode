package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems.Color.LEDSubsystem;

import java.security.InvalidParameterException;

public class SubsystemCollection {
    private static SubsystemCollection instance = null;
    private HardwareMap map = null;

    public final MecanumDriveBase drivebase;
    public final IntakeElevatorSubsystem intake;
    public final CarouselSubsystem hopper;
    public final ShooterSubsystem shooter;
    public final LEDSubsystem led;

    public final GamepadEx Gamepad1;
    public final GamepadEx Gamepad2;

    private SubsystemCollection(HardwareMap map, Gamepad gamepad1, Gamepad gamepad2) {
        this.map = map;

        drivebase = new MecanumDriveBase(this.map);
        intake = new IntakeElevatorSubsystem(this.map);
        hopper = new CarouselSubsystem(this.map);
        shooter = new ShooterSubsystem(this.map);
        led = new LEDSubsystem(this.map);

        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);
    }

    public static SubsystemCollection getInstance(HardwareMap map) {
        if (instance == null) {
            if (map == null) {
                throw new InvalidParameterException();
            }
        }

        return instance;
    }

    public void periodic() {
        drivebase.periodic();
//        intake.periodic();
        hopper.periodic();
//        shooter.periodic();

        this.Gamepad1.readButtons();
        this.Gamepad2.readButtons();
    }

    public static void deinit() {
        instance = null;
    }
}
