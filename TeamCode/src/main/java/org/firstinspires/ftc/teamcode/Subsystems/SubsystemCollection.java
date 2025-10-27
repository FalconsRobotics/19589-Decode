package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.security.InvalidParameterException;

public class SubsystemCollection {
    private static SubsystemCollection instance = null;
    private HardwareMap map = null;

    public final MecanumDriveBase drivebase;
    public final IntakeElevatorSubsystem intake;
    public final CarouselSubsystem hopper;
    public final ShooterSubsystem shooter;

    private SubsystemCollection(HardwareMap map) {
        this.map = map;

        drivebase = new MecanumDriveBase(this.map);
        intake = new IntakeElevatorSubsystem(this.map);
        hopper = new CarouselSubsystem(this.map);
        shooter = new ShooterSubsystem(this.map);
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
    }

    public static void deinit() {
        instance = null;
    }
}
