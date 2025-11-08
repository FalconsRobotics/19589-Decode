package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Arrays;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    public DrivebaseSubsystem driveInstance = null;
    public Limelight3A ll;
    public LLResult result;
    public Pose2D pose;

    // Cached MT2 pose (for stable telemetry/fallback)
    private Pose2D lastLLPose2d = null;
    private long   lastLLPoseMillis = 0L;

    //Motif value
    int motif;

    public VisionSubsystem(HardwareMap map, DrivebaseSubsystem suppliedDrive) {
        this.driveInstance = suppliedDrive;

        ll = map.get(Limelight3A.class, "limelight");
        ll.setPollRateHz(60);
        ll.pipelineSwitch(0);
        ll.start();
    }

    @Override
    public void periodic() {
        result = ll.getLatestResult();
        ll.updateRobotOrientation(driveInstance.odo.getHeading(AngleUnit.DEGREES));

        // Cache most recent valid MT2 2D pose
        if (result != null && result.isValid()) {
            Pose3D mt2 = result.getBotpose_MT2();
            if (mt2 != null) {
                lastLLPose2d = new Pose2D(
                        DistanceUnit.MM,
                        mt2.getPosition().x,
                        mt2.getPosition().y,
                        AngleUnit.DEGREES,
                        driveInstance.odo.getHeading(AngleUnit.DEGREES) // keep heading from odometry
                );
                lastLLPoseMillis = System.currentTimeMillis();
            }
        }
    }

    // ----------------------------
    // Limelight helpers (for commands/telemetry)
    // ----------------------------

    public double geTx(){
        return result.getTx();
    }

    /** returns the motif value*/
    public int getMotif(){
        return motif;
    }

    /** Finds the tag id and sets motif to value 1, 2, or 3 depending on tag. Motif value of 0 means no tag was detected*/
    public void findMotif(){
        ll.pipelineSwitch(0);
        List<LLResultTypes.FiducialResult> fidRes = result.getFiducialResults();

        for(LLResultTypes.FiducialResult fid : fidRes){
            int id = fid.getFiducialId();
            if(id == 21){
                motif = 1;
            } else if(id == 22){
                motif = 2;
            } else if(id == 23){
                motif = 3;
            } else {
                motif = 0;
            }
        }
    }

    public Pose2D llPoseMT1(){
        ll.pipelineSwitch(1);
        if (result != null && result.isValid()) {
            Pose3D p3 = result.getBotpose();
            if (p3 != null) {
                return new Pose2D(
                        DistanceUnit.MM,
                        p3.getPosition().x,
                        p3.getPosition().y,
                        AngleUnit.DEGREES,
                        driveInstance.odo.getHeading(AngleUnit.DEGREES)
                );
            }
        }
        return lastLLPose2d;
    }

    /** Best-available 2D field pose from MegaTag2 this loop, or last cached. Units: mm & deg. */
    public Pose2D llPose() {
        ll.pipelineSwitch(1);
        if (result != null && result.isValid()) {
            Pose3D p3 = result.getBotpose_MT2();
            if (p3 != null) {
                return new Pose2D(
                        DistanceUnit.MM,
                        p3.getPosition().x,
                        p3.getPosition().y,
                        AngleUnit.DEGREES,
                        driveInstance.odo.getHeading(AngleUnit.DEGREES)
                );
            }
        }
        return lastLLPose2d;
    }

    /** Age of the cached MT2 pose in milliseconds (Long.MAX_VALUE if none yet). */
    public long llPoseAgeMillis() {
        return (lastLLPoseMillis == 0) ? Long.MAX_VALUE : (System.currentTimeMillis() - lastLLPoseMillis);
    }

    /** Angle (deg, [-180,180]) from robot to the *closest* tag among preferred IDs. NaN if none visible. */
    public double angleToGoal(int... preferredTagIds) {
        if (result == null || !result.isValid()) return Double.NaN;
        List<LLResultTypes.FiducialResult> fidRes = result.getFiducialResults();
        if (fidRes == null || fidRes.isEmpty()) return Double.NaN;

        boolean filter = (preferredTagIds != null && preferredTagIds.length > 0);
        LLResultTypes.FiducialResult best = null;
        double bestDistSq = Double.POSITIVE_INFINITY;

        for (LLResultTypes.FiducialResult fr : fidRes) {
            if (filter && Arrays.stream(preferredTagIds).noneMatch(id -> id == fr.getFiducialId()))
                continue;

            Pose3D p = fr.getTargetPoseRobotSpace();
            if (p == null) p = fr.getTargetPoseCameraSpace();
            if (p == null) continue;

            double x = p.getPosition().x; // forward
            double y = p.getPosition().y; // left
            double d2 = x*x + y*y;

            if (d2 < bestDistSq) {
                bestDistSq = d2;
                best = fr;
            }
        }

        if (best == null) return Double.NaN;

        Pose3D p = best.getTargetPoseRobotSpace();
        if (p == null) p = best.getTargetPoseCameraSpace();

        double x = p.getPosition().x; // forward
        double y = p.getPosition().y; // left
        double angleDeg = Math.toDegrees(Math.atan2(y, x));
        // normalize to [-180, 180]
        angleDeg = ((angleDeg + 180) % 360 + 360) % 360 - 180;
        return angleDeg;
    }

    /** Convenience for FTC DECODE goals (edit IDs if your field differs). */
    public double angleToAllianceGoal(boolean isRed) {
        // Common mapping you mentioned previously: Red=20, Blue=24
        return isRed ? angleToGoal(20) : angleToGoal(24);
    }

    // ----------------------------
    // Heading/odometry access
    // ----------------------------

    public double getHeadingDegrees() {
        return driveInstance.odo.getHeading(AngleUnit.DEGREES);
    }

    /** Resets Pinpoint pose & IMU (re-zeros heading & odometry). */
    public void resetHeading() {
        driveInstance.odo.resetPosAndIMU();
        driveInstance.turnPID.reset();
        driveInstance.turnPID.setSetPoint(0);
    }

    /** Optional: snap helper for button-driven heading locks (pairs with driveFieldCentricHeadingLock). */
    public void snapHeadingDegrees(double headingDeg) {
        driveInstance.turnPID.setSetPoint(0); // we pass error directly in driveFieldCentricHeadingLock
        // no persistent lock here; the caller supplies lockHeading to the drive method below
    }
}
