package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.controller.Controller;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import java.util.List;

@TeleOp(name = "Main")
public class Main extends OpMode {
    public MecanumDriveBase drivebase;
    public IntakeElevatorSubsystem intake;
    public CarouselSubsystem hopper;
    public ShooterSubsystem shooter;

    public GamepadEx Gamepad1 = new GamepadEx(gamepad1);
    public GamepadEx Gamepad2 = new GamepadEx(gamepad2);

    @Override
    public void init() {
        drivebase = new MecanumDriveBase(hardwareMap);
        intake = new IntakeElevatorSubsystem(hardwareMap);
        hopper = new CarouselSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {
        // Variables to store the x (Left X), y (Left Y, reversed), and angle for turning (Right X).
        double cX = Gamepad1.getLeftX();
        double cY = -Gamepad1.getLeftY();
        double cA = Gamepad1.getRightX();

        // Drive using robot-centric mode, feeding in the controller x, y, and angle.
        // Heading is set to -1 because the functionality is not implemented, just to
        // keep it clear.
        drivebase.Drive(cX, cY, cA, -1);

        if (gamepad1.dpad_up) drivebase.DriveFieldCentric(0, 1, 0, -1);
        if (gamepad1.dpad_down) drivebase.DriveFieldCentric(0, -1, 0, -1);
        if (gamepad1.dpad_left) drivebase.DriveFieldCentric(-1, 0, 0, -1);
        if (gamepad1.dpad_right) drivebase.DriveFieldCentric(1, 0, 0, -1);
    }
}
