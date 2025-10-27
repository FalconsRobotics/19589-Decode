package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.Drivebase.DriveFieldCentricCommand;
import org.firstinspires.ftc.teamcode.Commands.Drivebase.DriveRobotCentricCommand;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeSetPowerCommand;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemCollection;

@Disabled
@TeleOp(name = "Main")
public class CommandTeleOp extends CommandOpMode {
    private SubsystemCollection sys;

    private GamepadEx Gamepad1;
    private GamepadEx Gamepad2;

    @Override
    public void initialize() {
        SubsystemCollection.deinit();
        sys = SubsystemCollection.getInstance(hardwareMap);

        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

//        register(sys.drivebase, sys.intake, sys.hopper, sys.shooter);

//        sys.drivebase.setDefaultCommand(new DriveRobotCentricCommand(Gamepad1::getLeftX, Gamepad1::getLeftY, Gamepad1::getRightX));

        Gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new DriveFieldCentricCommand(0.0, 1.0, 0.0));
        Gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new DriveFieldCentricCommand(0.0, -1.0, 0.0));
        Gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileHeld(new DriveFieldCentricCommand(-1.0, 0.0, 0.0));
        Gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whileHeld(new DriveFieldCentricCommand(1.0, 0.0, 0.0));

        Gamepad1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new ConditionalCommand(
                        new IntakeSetPowerCommand(1.0),
                        new IntakeSetPowerCommand(0.0),
                        () -> sys.intake.isIntakeActive
                )
        );
    }
}
