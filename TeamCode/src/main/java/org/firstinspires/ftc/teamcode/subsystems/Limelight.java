package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import

public class Limelight {
    Limelight3A limelight;
    LLResult result;
    int motif = 0;

    public Limelight(HardwareMap map){
        limelight = map.get(Limelight3A.class, "limelight");
        result = limelight.getLatestResult();
    }

    //sets motif variable to 1, 2, or 3 depending on id seen
    //if no id is seen, motif is automatically set to 3
    private void findMotif(){
        limelight.pipelineSwitch(0);
        limelight.getLatestResult();
        int id;
        if(result != null && result.isValid()){
            id = result.getFiducialResults().get(0).getFiducialId();
        } else{
            id = 0;
        }
        if(id == 21){
            motif = 1;
        } else if(id == 22){
            motif = 2;
        } else {
            motif = 3;
        }
    }

    //returns angle X to the center of shooting goal(Blue Alliance)
    public double angleToGoalBlue(){
        limelight.pipelineSwitch(1);
        limelight.getLatestResult();
        if(result != null && result.isValid()){
            return result.getTx();
        } else {
            return 0;
        }
    }

    //returns angle X to the center of shooting goal(Red Alliance)
    public double angleToGoalRed(){
        limelight.pipelineSwitch(2);
        limelight.getLatestResult();
        if(result != null && result.isValid()){
            return result.getTx();
        } else {
            return 0;
        }
    }

    //ballPos returns an x and y value to the closest ball it can see
    //ballPos is an array with index 0 being X & index 1 being Y
    public double[] findClosestBall(){
        double[] llpython = result.getPythonOutput();
        double[] ballPos = new double[2];

        if(llpython.length > 0){
            ballPos[0] = llpython[1];
            ballPos[1] = llpython[2];
        } else {
            ballPos[0] = 0;
            ballPos[1] = 0;
        }
        return ballPos;
    }

    class BallDetected{
        int x, y, area, color;
    }
    public double[] findClosestPurpleBall(){
        double[] llpython = result.getPythonOutput();
        double[] ballPos = new double[2];

        int numBalls = llpython.length/4;

        for(int i=0; i < numBalls; i++){
            double color = llpython[i*4];
            if(color == 1.0){
                ballPos[0] = llpython[i*4+1];
                ballPos[1] = llpython[i*4+2];
                return ballPos;
            } else {
                ballPos[0] = 0;
                ballPos[1] = 0;
            }
        }
        return ballPos;
    }

    public double[] findClosestGreenBall(){
        double[] llpython = result.getPythonOutput();
        double[] ballPos = new double[2];

        int numBalls = llpython.length/4;

        for(int i=0; i < numBalls; i++){
            double color = llpython[i*4];
            if(color == 2.0){
                ballPos[0] = llpython[i*4+1];
                ballPos[1] = llpython[i*4+2];
                return ballPos;
            } else {
                ballPos[0] = 0;
                ballPos[1] = 0;
            }
        }
        return ballPos;
    }
}
