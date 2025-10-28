package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.SubsystemCollection;

import java.util.List;

public class Main extends OpMode {
    SubsystemCollection sys;
    

    @Override
    public void init() {
        // A class that contains all of the subsystems.
        // It is used as a singleton to make sure that every OpMode
        // and (future) Command references the same subsystem instance.
        SubsystemCollection.deinit();
        sys = SubsystemCollection.getInstance(hardwareMap);

        // Enable bulk cached reading to speed up I2C read times.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void start() {
        // As soon as the match starts (not when we init, when the match actually starts
        // and we're allowed to move), we want the shooter to get up to speed.
//        sys.shooter.setPower(1.0);
    }

    @Override
    public void loop() {
        // Call the periodic functions from the SubsystemCollection
        // class. Functions that need to be run every loop.
        sys.periodic();

        //region Driver Functions (with sys.sys.Gamepad1)

        // Gather joystick values from LX, LY (inverted, so up is forward y),
        // and RX (for turning) on sys.Gamepad1.
        double cX = sys.Gamepad1.getLeftX();
        double cY = -sys.Gamepad1.getLeftY();
        double cA = sys.Gamepad1.getRightX();

        // Use the trigger for manual speed control, added on top of the joystick magnitude.
        // This is set to the sys.Gamepad1 right trigger.
        double speedMultiplier = sys.Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        // Robot-centric drive with the joystick inputs from earlier lines.
        // I multiplied by speedMultiplier to allow using the trigger.
        sys.drivebase.Drive(cX * speedMultiplier, cY * speedMultiplier, cA * speedMultiplier, -1);

        // Field-centric drive using the D-Pad inputs. Up goes up (y+), left goes left (x-), etc.
        // Maybe should have braces for each case. However, you're never going to press more than
        // one D-Pad input at a time.
        if (sys.Gamepad1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) sys.drivebase.DriveFieldCentric(0, 1, 0, -1);
        if (sys.Gamepad1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) sys.drivebase.DriveFieldCentric(0, -1, 0, -1);
        if (sys.Gamepad1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) sys.drivebase.DriveFieldCentric(-1, 0, 0, -1);
        if (sys.Gamepad1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) sys.drivebase.DriveFieldCentric(1, 0, 0, -1);

        //endregion

        //region Utility Functions (with sys.Gamepad2)

        if (sys.Gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            sys.hopper.setCounter(sys.hopper.getCounter() - 1);
        } else if (sys.Gamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            sys.hopper.setCounter(sys.hopper.getCounter() + 1);
        } else if (sys.Gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            sys.hopper.setCounter(0);
        }


        telemetry.addLine("------------------------------");
        telemetry.addData("Counter", sys.hopper.getCounter());
        telemetry.addData("Servo Position", sys.hopper.getPosDouble());
        telemetry.addLine("------------------------------");
        telemetry.update();
        //endregion
    }

    @Override
    public void stop() {

    }
}
