package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;

@TeleOp(name = "Hopper Testing", group = "Test")
public class HopperTesting extends LinearOpMode {
    public HopperSubsystem hopper;
    public GamepadEx gp;


    public void runOpMode() {
        hopper = new HopperSubsystem(hardwareMap);
        gp = new GamepadEx(gamepad1);


        while (opModeInInit()) {

        }

        while (opModeIsActive()) {



            if (gp.wasJustPressed(GamepadKeys.Button.A)) {
                hopper.toPosition(0);
            }
            else {
                hopper.setServoPower(
                        0.0
                                + (gp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.5)
                                - (gp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.5)
                );
            }


            gp.readButtons();

            telemetry.addData("Encoder Position", hopper.getServoPosition());
            telemetry.addData("Servo Power", hopper.getServoPower());
            telemetry.update();
        }
    }
}
