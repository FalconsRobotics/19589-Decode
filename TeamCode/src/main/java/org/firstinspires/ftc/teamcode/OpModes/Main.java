package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.SubsystemCollection;

public class Main extends OpMode {
    SubsystemCollection sys;

    @Override
    public void init() {
        SubsystemCollection.deinit();
        sys = SubsystemCollection.getInstance(hardwareMap);
    }

    @Override
    public void loop() {
        sys.periodic();

        double cX = gamepad1.left_stick_x;
        double cY = -gamepad1.left_stick_y;
        double cA = gamepad1.right_stick_x;

        sys.drivebase.Drive(cX, cY, cA, -1);

        if (gamepad1.dpad_up) sys.drivebase.DriveFieldCentric(0, 1, 0, -1);
        if (gamepad1.dpad_down) sys.drivebase.DriveFieldCentric(0, -1, 0, -1);
        if (gamepad1.dpad_left) sys.drivebase.DriveFieldCentric(-1, 0, 0, -1);
        if (gamepad1.dpad_right) sys.drivebase.DriveFieldCentric(1, 0, 0, -1);
    }

    @Override
    public void stop() {

    }
}
