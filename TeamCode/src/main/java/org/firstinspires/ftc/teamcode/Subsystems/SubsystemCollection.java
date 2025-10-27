package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Color.LedSubsystem;

import java.security.InvalidParameterException;

public class SubsystemCollection {
    private static SubsystemCollection instance = null;
    private HardwareMap map = null;

    public final MecanumDriveBase drivebase;
    public final IntakeElevatorSubsystem intake;
    public final HopperSubsystem hopper;
    public final ShooterSubsystem shooter;
    public final LedSubsystem led;

    private SubsystemCollection(HardwareMap map) {
        this.map = map;

        drivebase = new MecanumDriveBase(this.map);
        intake = new IntakeElevatorSubsystem(this.map);
        hopper = new HopperSubsystem(this.map);
        shooter = new ShooterSubsystem(this.map);
        led = new LedSubsystem(this.map);
    }

    public static SubsystemCollection getInstance(HardwareMap map) {
        if (instance == null) {
            if (map == null) {
                throw new InvalidParameterException();
            }
        }

        return instance;
    }

    public static void deinit() {
        instance = null;
    }
}
