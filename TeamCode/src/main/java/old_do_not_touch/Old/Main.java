
package old_do_not_touch.Old;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import old_do_not_touch.Subsystems.CarouselSubsystem;
import old_do_not_touch.Subsystems.ShooterSubsystem;
import old_do_not_touch.Subsystems.IntakeElevatorSubsystem;

@Disabled
@TeleOp(name = "Tele-Op")
public class Main extends OpMode {
    private SwerveSubsystem swerve = new SwerveSubsystem(hardwareMap);
    private IntakeElevatorSubsystem intake = new IntakeElevatorSubsystem(hardwareMap);
    private CarouselSubsystem carousel = new CarouselSubsystem(hardwareMap);
    private ShooterSubsystem extake = new ShooterSubsystem(hardwareMap);

    @Override
    public void init() {
        //initializing all hardware
        swerve.mapHardware(hardwareMap);
        DriveConstants.powerMult = 1;
    }

    @Override
    public void loop() {
        //variables to store the joystick positions
        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double a = -gamepad1.right_stick_x;

        swerve.Drive(x,y,a, -1);

        if(Math.abs(gamepad1.right_stick_x) > 0.05) swerve.Drive(x,y,a,-1);
        else if(gamepad1.dpad_up) swerve.Drive(x,y,a,0);
        else if(gamepad1.dpad_left) swerve.Drive(x,y,a,90);
        else if(gamepad1.dpad_down) swerve.Drive(x,y,a,180);
        else if(gamepad1.dpad_right) swerve.Drive(x,y,a,270);

        telemetry.addData("Power:", DriveConstants.powerMult); //right -y
        telemetry.addData("Velocity:", swerve.odo.getVelY(DistanceUnit.MM)); //right -y
        telemetry.addData("Update Time:", swerve.getLoopTime(getRuntime()));
        telemetry.update();
    }

    @Override
    public void stop(){

    }
}
