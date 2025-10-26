package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayList;
import java.util.List;

//sets the teleop name visable on the driver station and sets it as a teleop program
public class SwerveSubsystem {
    DcMotor steeringMotor;
    DcMotor driveA, driveB, driveC;
    public GoBildaPinpointDriver odo;
    private HardwareMap hardwareMap;

    //Publicly used variables
    double powerDir = 1; // Changes the direction of power for the wheels
    int ticksToMove = 0; // Difference between where you are and need to be for rotating swerve modules in ticks
    double joyStickAngle = 0; // Angle of left joystick to create vector
    double wheelFlip = 0; // Flips the angle of the wheels to account for flipping
    boolean latch = false, timeLatch = false; // Locks functions once activated
    double lastHeading = 0; // Last heading once you let go of joystick for heading correction
    double normalizedIMUBotHeading = 0; //-180 to 180 bot heading
    double botHeadingVel = 0; // Turning velocity
    double holdDelta = 0; // Angle difference between last heading and current heading

    //TODO: look over auto variables
    double oldTime = 0;

    int currentPointIndex = 0;
    double p = 0;

    double speed = 0;
    double xOutput = 0;
    double yOutput = 0;
    double error = 0;
    double targetTime = 0, deltaTime = 0;

    public SwerveSubsystem(HardwareMap map){
        this.hardwareMap = map;
        steeringMotor = hardwareMap.get(DcMotor.class, "SteeringMotor");
        driveA = hardwareMap.get(DcMotor.class, "LeftMotor");
        driveB = hardwareMap.get(DcMotor.class, "RightMotor");
        driveC = hardwareMap.get(DcMotor.class, "BackMotor");

        // Odometry initialization
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.recalibrateIMU();

        // Steering motor settings
        steeringMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        steeringMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Enables auto bulk reads to speed up code
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    // function to map hardware called on init
    public void mapHardware(HardwareMap map){
        this.hardwareMap = map;
        steeringMotor = hardwareMap.get(DcMotor.class, "SteeringMotor");
        driveA = hardwareMap.get(DcMotor.class, "LeftMotor");
        driveB = hardwareMap.get(DcMotor.class, "RightMotor");
        driveC = hardwareMap.get(DcMotor.class, "BackMotor");

        // Odometry initialization
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.recalibrateIMU();

        // Steering motor settings
        steeringMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        steeringMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Enables auto bulk reads to speed up code
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public double getX(){
        return odo.getPosX(DistanceUnit.INCH);
    }
    public double getY(){
        return -odo.getPosY(DistanceUnit.INCH);
    }
    public double getLoopTime(double newTime){ // input getRunTime() function to this
        // Calculation to get code loop times
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;
        return frequency;
    }

    public double normalizeTo180 (double angle) {
        return ((angle + 540) % 360) - 180;
    }

    public void recenterModules(){
        steeringMotor.setTargetPosition(0);
        steeringMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //TODO: change default to this instead of encoder
        steeringMotor.setPower(DriveConstants.steeringPower);
    }
    public void Drive(double joyStickX, double joyStickY, double joyStickRotation, double targetRobotHeading) {
        //updates pinpoint
        odo.update();

        double IMUBotHeading = odo.getHeading(UnnormalizedAngleUnit.DEGREES);
        normalizedIMUBotHeading = (IMUBotHeading + 360) % 360;
        double IMUBotHeadingRad = Math.toRadians(IMUBotHeading);

        botHeadingVel = Math.abs(odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));

        // Rotated x and y variables to allow for field centric driving
        // Joystick inputs transformed to robot coordinates for field centric driving
        double rotX = joyStickX * Math.cos(IMUBotHeadingRad) - joyStickY * Math.sin(IMUBotHeadingRad);
        double rotY = joyStickX * Math.sin(IMUBotHeadingRad) + joyStickY * Math.cos(IMUBotHeadingRad);

        // Power for the wheels based off the magnitude of the joysticks movement then making sure it doesnt go above 1
        double power = Math.sqrt((joyStickX * joyStickX)+(joyStickY * joyStickY)); //TODO check if these want to be rotX/Y
        if(power > 1) power = 1;

        // Angle of the joystick calculation and then normalize angle of joystick to be in 0 to 360 degrees
        if(joyStickX != 0 || joyStickY != 0) joyStickAngle = Math.toDegrees(Math.atan2(rotX, rotY));
        if (joyStickAngle < 0) joyStickAngle += 360;

        // If joysticks are sitting still set power for steering motor to 0 to conserve power
        if (joyStickX <= 0.05 && joyStickY <= 0.05 &&  ticksToMove < 15) steeringMotor.setPower(0);

        //TODO: tune heading velocity constraint, strength and fix constant spinning
        // Correction so if not moving right joystick it holds heading to account for drift
        if (Math.abs(joyStickRotation) <= 0.05){
            if(!latch && botHeadingVel < 0.2 * DriveConstants.powerMult){
                lastHeading = normalizedIMUBotHeading;
                latch = true;
            }else if(latch){
                if(targetRobotHeading != -1) lastHeading = targetRobotHeading;
                holdDelta = normalizeTo180(lastHeading - normalizedIMUBotHeading);
                if(Math.abs(holdDelta) > 2) joyStickRotation = holdDelta * DriveConstants.turningGainP;
            }
        }else latch = false;

        // Current swerve module angle calculation
        double unnormalizedWheelAngle = (steeringMotor.getCurrentPosition() / (DriveConstants.ticksPerRev * DriveConstants.gearRatio)) * 360;

        // Current swerve module angle normalized to -180 to 180
        double currentWheelAngle = normalizeTo180(unnormalizedWheelAngle);

        // Current swerve module angle as seen by robot with flip correction in a -360 to 360
        double flippedWheelAngle = unnormalizedWheelAngle + wheelFlip;
        flippedWheelAngle %= 360;


        // Difference between current wheel angle and target wheel angle
        double angleDelta = normalizeTo180(joyStickAngle - flippedWheelAngle);
        wheelFlip %= 360;

        // If difference in angles is too large flips direction of wheels so the turning is more efficient
        if(Math.abs(angleDelta) >= DriveConstants.flipPoint){
            wheelFlip += 180 * Math.signum(angleDelta);
            powerDir *= -1;
        }

        //re-calculates the wheel angle and delta after the corrections
        flippedWheelAngle = ((steeringMotor.getCurrentPosition() / (DriveConstants.ticksPerRev * DriveConstants.gearRatio)) * 360) + wheelFlip;
        flippedWheelAngle %= 360;

        angleDelta = normalizeTo180(joyStickAngle - flippedWheelAngle);

        //calculates the target motor position to turn the wheels
        double targetPos = (angleDelta / 360) * DriveConstants.ticksPerRev * DriveConstants.gearRatio;
        ticksToMove = steeringMotor.getCurrentPosition() + (int)Math.round(targetPos);

        //turns the module to set position
        steeringMotor.setTargetPosition(ticksToMove);
        steeringMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //TODO Change to default?
        steeringMotor.setPower(DriveConstants.steeringPower);

        //basically tank drive power
        double leftWheel = (power * powerDir + joyStickRotation) * DriveConstants.powerMult;
        double rightWheel = (power * powerDir - joyStickRotation) * DriveConstants.powerMult;
        double middleWheel = (power * powerDir) * DriveConstants.powerMult;
        //sets drive power on wheels depending on the direction that the swerve modules are facing to account for the turning
        //2
        if(currentWheelAngle > 30 && currentWheelAngle <= 90){
            driveA.setPower(rightWheel); //
            driveB.setPower(leftWheel); //
            driveC.setPower(middleWheel);
        }
        else if(currentWheelAngle <= -90 && currentWheelAngle > -150){
            driveA.setPower(leftWheel); //
            driveB.setPower(rightWheel); //
            driveC.setPower(middleWheel);
        }//3
        else if (currentWheelAngle > 90 && currentWheelAngle <= 150){
            driveA.setPower(rightWheel); //
            driveB.setPower(middleWheel);
            driveC.setPower(leftWheel); //
        }else if(currentWheelAngle <= -30 && currentWheelAngle > -90){
            driveA.setPower(leftWheel); //
            driveB.setPower(middleWheel);
            driveC.setPower(rightWheel); //
        }//1
        else if(currentWheelAngle > -30 && currentWheelAngle <= 30){
            driveA.setPower(middleWheel);
            driveB.setPower(leftWheel); //
            driveC.setPower(rightWheel); //
        }else{
            driveA.setPower(middleWheel);
            driveB.setPower(rightWheel); //
            driveC.setPower(leftWheel); //
        }
    }

    //class for points on the path for auto pathing
    public static class PathPoint {
        public double x, y, heading, precision, waitMS;
        public Runnable action;

        //path point callouts to allow for usage of different variables without others
        public PathPoint(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.precision = DriveConstants.defaultPrecision;
            this.action = null;
            this.waitMS = 0;
        }
        public PathPoint(double x, double y, double heading, double precision) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.precision = precision;
            this.action = null;
            this.waitMS = 0;
        }
        public PathPoint(Runnable action, double waitMS){
            this.action = action;
            this.waitMS = waitMS;
            this.precision = DriveConstants.defaultPrecision;
        }
        public PathPoint(double x, double y, double heading, Runnable action){
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.precision = DriveConstants.defaultPrecision;
            this.action = action;
            this.waitMS = 0;
        }
        public PathPoint(double x, double y, double heading, double precision, Runnable action){
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.precision = precision;
            this.action = action;
            this.waitMS = 0;
        }
    }
    // Path class (inner class inside Functions)
    public class Path {
        List<PathPoint> points;

        public Path() {
            points = new ArrayList<>();
        }

        //new point that will hold current heading but drive to point
        public void newPoint(double x, double y){
            points.add(new PathPoint(x, y, -1));
        }
        //new point that can drive to a point and turn robot to a heading
        public void newPoint(double x, double y, double heading){
            points.add(new PathPoint(x, y, heading));
        }
        //new point to drive to point with heading and a specified precision
        public void newPoint(double x, double y, double heading, double precision){
            points.add(new PathPoint(x, y, heading, precision));
        }
        //new point to add a action where robot stops driving and waits
        public void newActionWait(Runnable action, double waitMS){
            points.add(new PathPoint(action, waitMS));
        }
        //new point where robot runs a action but keeps driving to point
        public void newDrivingAction(double x, double y, double heading, Runnable action){
            points.add(new PathPoint(x, y, heading, action));
        }
        //new action where robot runs a action but keeps driving to point but has precision added in
        public void newDrivingAction(double x, double y, double heading, double precision, Runnable action){
            points.add(new PathPoint(x, y, heading, precision, action));
        }
    }
    //function to follow list of points called a path point by point
    public void followPath(Path path, double runtime){

        //retrieves the current path point that you are going to
        PathPoint currentPoint = path.points.get(currentPointIndex);

        //retrieves current data from path point
        double waitTimeMs = currentPoint.waitMS;
        Runnable action = currentPoint.action;

        //current x and y coordinated from odometry
        double curX = odo.getPosX(DistanceUnit.INCH);
        double curY = odo.getPosY(DistanceUnit.INCH);

        //flips x and y so +x is right and +y is forwards
        double driveX = -curY;
        double driveY = curX;

        //checks if there is a time value given and if there is a time value calculates the end time we want to go to and then the difference between current time and end time
        if(!timeLatch && runtime != 0){
            targetTime = runtime + waitTimeMs;
            deltaTime = runtime - targetTime;
            timeLatch = true;
        }
//        else if(deltaTime <= 0 && action == null) timeLatch = false;

        //calculated error in time
        deltaTime = targetTime - runtime;

        //Checks if action exists and if it does runs it
        if (action != null) {
            //if a action exists and there is a wait sets the x,y, and heading to current values to correct for no the empty values in the functions
            if(runtime > 0){ //TODO change to delta time? ignore maybe.
                currentPoint.x = driveX;
                currentPoint.y = driveY;
                currentPoint.heading = odo.getHeading(AngleUnit.DEGREES);
            }
            action.run();

        }

        //if there is no time value given or if the delta time is less than or equal to 0 runs drive process otherwise keeps robot still
        if(deltaTime <= 0 || runtime == 0) {
            //retrieves variables from current path point
            double x = currentPoint.x;
            double y = currentPoint.y;
            double heading = currentPoint.heading;
            double precision = currentPoint.precision;

            //finds difference between the targeted x and y and the current x and y coordinates
            double dx = x - driveX;
            double dy = y - driveY;

            //finds error distance from dx and dy
            error = Math.sqrt((dx * dx) + (dy * dy));

            //if the error distance is less than the inputted precision and on last point stops the robot and recenters modules
            if(error <= precision && currentPointIndex + 1 == path.points.size()){
                stop();
                recenterModules();
            }//if error distance is greater than the inputted precision runs the drive code
            else if (error > precision) {
                //calculated proportional  aspect of speed based off the error times a constant
                p = error * DriveConstants.driveToPointGainP;

                // Speed factor, proportional to distance with a feed forward added in as the minimum power needed to move motors so it can always move closer
                speed = Math.min(p + DriveConstants.driveToPointF, 1.0);

                //if you are not on the last point and the next point is not a wait then set speed to 1 to go full speed through points
                if(currentPointIndex + 1 < path.points.size() && path.points.get(currentPointIndex + 1).waitMS == 0) speed = 1;

                //calculates the outputting fake "joystick" outputs to input into drive function to move robot
                xOutput = (-dx / error) * speed;
                yOutput = (dy / error) * speed;

                //calls drive function with the output variables and has 0 for a since heading is purely controlled by the automatic heading
                Drive(xOutput, yOutput, 0, heading);
            }//if you are within range but not on last point goes to next point and resets the time latch to allow for new wait timer
            else if (currentPointIndex + 1 < path.points.size()){
                timeLatch = false;
                currentPointIndex++;
            }
            //if delta time is still greater than 0 then robot stops and does not move
        }else stop();
    }

    //function to stop all wheels keeping it from moving
    public void stop(){
        driveA.setPower(0);
        driveB.setPower(0);
        driveC.setPower(0);
    }
}

