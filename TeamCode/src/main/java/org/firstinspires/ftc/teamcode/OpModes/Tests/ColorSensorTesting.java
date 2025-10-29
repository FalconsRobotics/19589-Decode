package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ControllerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselSubsystem;

@Disabled
@TeleOp
public class ColorSensorTesting extends LinearOpMode {
    private FtcDashboard dashboard;
    public CarouselSubsystem hopper;
    public ControllerSubsystem control;
    // ... other variables

    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        hopper = new CarouselSubsystem(hardwareMap);
        control = new ControllerSubsystem(gamepad1, gamepad2);

        while (opModeInInit()) {
            if (control.base.isDown(GamepadKeys.Button.A)) {
                dashboard.getTelemetry().addData("greenR", hopper.colorSensorRed());
                dashboard.getTelemetry().addData("greenG", hopper.colorSensorGreen());
                dashboard.getTelemetry().addData("greenB", hopper.colorSensorBlue());

                dashboard.getTelemetry().update();
            }
        }

        while (opModeIsActive()) {
            if (control.base.isDown(GamepadKeys.Button.A)) {
                dashboard.getTelemetry().addData("purpleR", hopper.colorSensorRed());
                dashboard.getTelemetry().addData("purpleG", hopper.colorSensorGreen());
                dashboard.getTelemetry().addData("purpleB", hopper.colorSensorBlue());

                dashboard.getTelemetry().update();
            }
        }
    }
}