package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeSetPowerCommand;
import org.firstinspires.ftc.teamcode.commands.PeriodicFunctionCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterSetSpeedCommand;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "Test Shooter")
public class ShooterTester extends CommandOpMode {

    public ShooterSubsystem shooter;

    GamepadEx Gamepad1, Gamepad2;

    public void initialize(){
        shooter = new ShooterSubsystem(hardwareMap);

        Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();

        schedule(new CommandBase() {
            @Override
            public void execute() {
                telemetry.update();

                dashboard.addData("Shooter Speed", shooter.getVelocity());

                dashboard.update();

            }
        });

        register(shooter);

        shooter.setDefaultCommand(new ShooterSetSpeedCommand(shooter, () -> ShooterConstants.SPEED));

    }

}
