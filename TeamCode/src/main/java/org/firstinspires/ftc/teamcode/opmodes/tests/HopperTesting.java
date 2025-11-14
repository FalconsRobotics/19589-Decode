package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.constants.HopperConstants.TICKS_PER_STEP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.constants.HopperConstants;
import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;

import java.math.BigInteger;

@TeleOp(name = "Hopper Testing", group = "Test") @Config
public class HopperTesting extends OpMode {

    public HopperSubsystem hopper;
    public GamepadEx gp;


    @Override
    public void init() {
        hopper = new HopperSubsystem(hardwareMap);
        GamepadEx gp = new GamepadEx(gamepad1);
    }

    @Override
    public void init_loop() {
        if (!hopper.isHomed) {
            if (!hopper.magnetIsActive()) {
                hopper.setServoPower(HopperConstants.HOMING_POWER);
            }
            else {
                hopper.setServoPower(0.0);
                hopper.isHomed = true;
            }
        }
        else {
            hopper.setServoPower(0.0);
            hopper.zeroEncoder();
        }
    }

    @Override
    public void start() {
        hopper.setServoPower(0.0);
    }

    @Override
    public void loop() {
        gp.readButtons();

        if (gp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            hopper.toPosition(hopper.getServoPosition() + (int) HopperConstants.TICKS_PER_STEP);
        }
        if (gp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            hopper.toPosition(hopper.getServoPosition() - (int) HopperConstants.TICKS_PER_STEP);
        }




        telemetry.addData("Raw Encoder", hopper.getServoPosition());
        telemetry.update();
    }

}
