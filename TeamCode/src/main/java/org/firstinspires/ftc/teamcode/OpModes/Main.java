package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.responses.HubType;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemCollection;

import java.util.List;

public class Main extends OpMode {
    SubsystemCollection sys;

    @Override
    public void init() {
        SubsystemCollection.deinit();
        sys = SubsystemCollection.getInstance(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void start() {
        // As soon as the match starts (not when we init, when the match actually starts
        // and we're allowed to move), we want the shooter to get up to speed.
        sys.shooter.setPower(1.0);
    }

    @Override
    public void loop() {
        // Call the periodic functions from the SubsystemCollection
        // class. Functions that need to be run every loop.
        sys.periodic();

        //region Driver Functions (with gamepad1)

        // Gather joystick values from LX, LY (inverted, so up is forward y),
        // and RX (for turning) on gamepad1.
        double cX = gamepad1.left_stick_x;
        double cY = -gamepad1.left_stick_y;
        double cA = gamepad1.right_stick_x;

        // Use the trigger for manual speed control, added on top of the joystick magnitude.
        // This is set to the gamepad1 right trigger.
        double speedMultiplier = gamepad1.right_trigger;

        // Robot-centric drive with the joystick inputs from earlier lines.
        // I multiplied by speedMultiplier to allow using the trigger.
        sys.drivebase.Drive(cX * speedMultiplier, cY * speedMultiplier, cA * speedMultiplier, -1);

        if (gamepad1.dpad_up) sys.drivebase.DriveFieldCentric(0, 1, 0, -1);
        if (gamepad1.dpad_down) sys.drivebase.DriveFieldCentric(0, -1, 0, -1);
        if (gamepad1.dpad_left) sys.drivebase.DriveFieldCentric(-1, 0, 0, -1);
        if (gamepad1.dpad_right) sys.drivebase.DriveFieldCentric(1, 0, 0, -1);

        //endregion

        //region Utility Functions (with gamepad2)

        if (gamepad2.left_bumper) sys.hopper.setCounter(sys.hopper.getCounter() - 1);
        else if (gamepad2.right_bumper) sys.hopper.setCounter(sys.hopper.getCounter() + 1);


        //endregion
    }

    @Override
    public void stop() {

    }
}
