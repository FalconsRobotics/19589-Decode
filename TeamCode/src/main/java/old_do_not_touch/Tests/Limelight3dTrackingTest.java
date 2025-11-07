package old_do_not_touch.Tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;

@Disabled
@TeleOp(name = "Limelight 3d Testing - Teigan", group = "Test")
public class Limelight3dTrackingTest extends LinearOpMode {

    Pose2D pose;
    LLResult result;

    public void runOpMode(){
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");
        ll.pipelineSwitch(0);
        ll.setPollRateHz(67);
        ll.start();
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.initialize();
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.recalibrateIMU();

        result = ll.getLatestResult();

        double robotAngle = odo.getHeading(AngleUnit.DEGREES);
        ll.updateRobotOrientation(robotAngle);
        if (result != null && result.isValid()) {
            Pose3D pose3d = result.getBotpose_MT2();
            if (pose3d != null) {
                Pose2D botpose_mt2 = new Pose2D(DistanceUnit.MM, pose3d.getPosition().x,pose3d.getPosition().y, AngleUnit.DEGREES, robotAngle);
                telemetry.addData("MT2 Location:", "(" + botpose_mt2.getX(DistanceUnit.MM) + ", " + botpose_mt2.getY(DistanceUnit.MM) + ")");
            }
        }
        telemetry.update();

    }
}
