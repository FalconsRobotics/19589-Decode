package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.ToggleButtonReader;

import org.firstinspires.ftc.teamcode.constants.ColorConstants;
import org.firstinspires.ftc.teamcode.constants.HopperConstants;
import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name = "Hopper Testing", group = "Test") @Config
public class HopperTesting extends LinearOpMode {

    public HopperSubsystem hopper;
    public IntakeSubsystem intake;
    public ToggleButtonReader intakeOn;
    public GamepadEx gp;

    public void runOpMode() {

        hopper = new HopperSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        gp = new GamepadEx(gamepad1);
        intakeOn = new ToggleButtonReader(gp, GamepadKeys.Button.A);



        while (opModeInInit()) {
            hopper.runToMagnetZero();
        }

        while (opModeIsActive()) {
            gp.readButtons();

            ColorConstants.Ball ballColor = ColorConstants.detetctedColor(hopper.getBottomColor());

            if (gp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.1) {
                intake.setIntakePower(1.0);
            }
            else intake.setIntakePower(0.0);

            if (gp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                hopper.intakeOneTick();
            }
            if (gp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                hopper.extakeOneTick();
            }

            telemetry.addData("Detected Color", ballColor);
            telemetry.update();
        }
    }

}
