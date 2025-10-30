package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test DB")
public class MotorTest extends OpMode {
    DcMotor fl, fr, bl, br;

    @Override public void init() {
        fl = hardwareMap.get(DcMotor.class, "FrontLeft");
        fr = hardwareMap.get(DcMotor.class, "FrontRight");
        bl = hardwareMap.get(DcMotor.class, "BackLeft");
        br = hardwareMap.get(DcMotor.class, "BackRight");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
//        fr.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override public void loop() {
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double r = gamepad1.left_stick_y;

        fl.setPower(lx);
        fr.setPower(lx);
        bl.setPower(lx);
        br.setPower(lx);
    }
}
