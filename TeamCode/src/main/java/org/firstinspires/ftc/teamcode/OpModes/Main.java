
package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveSubsystem;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

@TeleOp(name = "Main")
public class Main extends OpMode {

    boolean toggle = false;
    private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    @Override
    public void init() {
        //initializing all hardware
        swerveSubsystem.mapHardware(hardwareMap);
        DriveConstants.powerMult = 1;
    }

    @Override
    public void loop() {
        //variables to store the joystick positions
        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double a = -gamepad1.right_stick_x;

        swerveSubsystem.Drive(x,y,a, -1);

        if(Math.abs(gamepad1.right_stick_x) > 0.05) swerveSubsystem.Drive(x,y,a,-1);
        else if(gamepad1.dpad_up) swerveSubsystem.Drive(x,y,a,0);
        else if(gamepad1.dpad_left) swerveSubsystem.Drive(x,y,a,90);
        else if(gamepad1.dpad_down) swerveSubsystem.Drive(x,y,a,180);
        else if(gamepad1.dpad_right) swerveSubsystem.Drive(x,y,a,270);

        telemetry.addData("Power:", DriveConstants.powerMult); //right -y
        telemetry.addData("Velocity:", swerveSubsystem.odo.getVelY(DistanceUnit.MM)); //right -y
        telemetry.addData("Update Time:", swerveSubsystem.getLoopTime(getRuntime()));
        telemetry.update();
    }

    @Override
    public void stop(){

    }
}
