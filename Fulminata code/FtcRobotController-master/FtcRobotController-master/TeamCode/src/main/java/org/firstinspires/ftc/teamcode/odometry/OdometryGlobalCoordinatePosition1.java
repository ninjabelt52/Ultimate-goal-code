package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class OdometryGlobalCoordinatePosition1 implements Runnable{
    //Odometry wheels
    private DcMotor verticalEncoderLeft, verticalEncoderRight, horizontalEncoder;

    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    private double robotLocalXCoordinatePosition = 0, robotLocalYCoordinatePosition = 0, robotOrientationRadians = 0, robotLocalOrientation = 0, robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotGlobalOrientation = 0;
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0, prevRobotOrentationRadians = 0;
    private double deltaX = 0, deltaY = 0;
    private double deltaXR = 0, deltaYR = 0;
    private double realXCoordinate = 0, realYcoordinate = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;
    private double horizontalRadius;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
    File horizontalRadiusFile = AppUtil.getInstance().getSettingsFile("horizontalRadius.txt");


    private int verticalLeftEncoderPositionMultiplier = 1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = 1;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param verticalEncoderLeft left odometry encoder, facing the vertical direction
     * @param verticalEncoderRight right odometry encoder, facing the vertical direction
     * @param horizontalEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    public OdometryGlobalCoordinatePosition1(DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, double COUNTS_PER_INCH, int threadSleepDelay){
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRight;
        this.horizontalEncoder = horizontalEncoder;
        sleepTime = threadSleepDelay;

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
        horizontalRadius = Double.parseDouble(ReadWriteFile.readFile(horizontalRadiusFile).trim());

    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
//    private void globalCoordinatePositionUpdate(){
//        //Get Current Positions
//        verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getCurrentPosition() * verticalLeftEncoderPositionMultiplier);
//        verticalRightEncoderWheelPosition = (verticalEncoderRight.getCurrentPosition() * verticalRightEncoderPositionMultiplier);
//
//        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
//        double rightChange = verticalRightEncoderWheelPosition - previousVerti calRightEncoderWheelPosition;
//
//        //Calculate Angle
//        changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
//        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));
//
//
//        //Get the components of the motion
//        normalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition()*normalEncoderPositionMultiplier);
//        double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
//        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);
//
//        double p = ((rightChange + leftChange) / 2);
//        double n = horizontalChange;
//        double r = p/changeInRobotOrientation;
//
//        double horizontalEncoder = normalEncoderWheelPosition - r*changeInRobotOrientation;
//
//
//        //Calculate and update the position values
//        robotLocalXCoordinatePosition = robotLocalXCoordinatePosition + (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));
//        robotLocalYCoordinatePosition = robotLocalYCoordinatePosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
//        robotLocalOrientation = robotOrientationRadians + prevRobotOrentationRadians;
//
//        robotGlobalXCoordinatePosition = r - robotLocalXCoordinatePosition;
//        robotGlobalYCoordinatePosition = robotLocalYCoordinatePosition;
//
//        deltaX = horizontalEncoder * Math.cos(changeInRobotOrientation);
//        deltaY = horizontalEncoder * Math.sin(changeInRobotOrientation);
//
//        deltaXR = robotGlobalXCoordinatePosition + deltaX;
//        deltaYR = robotGlobalYCoordinatePosition + deltaY;
//
//        realXCoordinate = deltaXR * Math.cos(robotLocalOrientation) - deltaYR * Math.sin(robotLocalOrientation);
//        realYcoordinate = deltaXR * Math.sin(robotLocalOrientation) + deltaYR * Math.cos(robotLocalOrientation);
//
//        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
//        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
//        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
//        prevRobotOrentationRadians = robotOrientationRadians;
//    }

    private double deltaTheta = 0;
    private static double globalTheta = 0;
    private static double prevDeltaTheta = 0;
    private double radius = 0;
    private double xPrime = 0, yPrime = 0;
    private double deltaX1 = 0, deltaY1 = 0, deltaX2 = 0, deltaY2 = 0;
    private double horizontalDistance = 0;
    private double totalXReal = 0, totalYReal = 0;
    private double totalX = 0, totalY = 0;

    private void globalCoordinatePositionUpdate(){
        verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getCurrentPosition() * verticalLeftEncoderPositionMultiplier);
        verticalRightEncoderWheelPosition = (verticalEncoderRight.getCurrentPosition() * verticalRightEncoderPositionMultiplier);
        normalEncoderWheelPosition = (horizontalEncoder.getCurrentPosition()*normalEncoderPositionMultiplier);


        deltaTheta = (verticalLeftEncoderWheelPosition - verticalRightEncoderWheelPosition) / robotEncoderWheelDistance;

        globalTheta = deltaTheta + prevDeltaTheta;

        radius = (verticalLeftEncoderWheelPosition - verticalRightEncoderWheelPosition) / deltaTheta;

        xPrime = (radius * Math.cos(deltaTheta));
        yPrime = radius * Math.sin(deltaTheta);

        deltaX1 = radius - xPrime;
        deltaY1 = yPrime;

        horizontalDistance = normalEncoderWheelPosition - deltaTheta * horizontalRadius;

        deltaX2 = horizontalDistance * Math.cos(deltaTheta);
        deltaY2 = horizontalDistance * Math.sin(deltaTheta);

        totalXReal = deltaX1 + deltaX2;
        totalYReal = deltaY1 + deltaY2;

        totalX = totalXReal * Math.cos(globalTheta) - totalYReal * Math.sin(globalTheta);
        totalY = totalXReal * Math.sin(globalTheta) + totalYReal * Math.cos(globalTheta);

        prevDeltaTheta = deltaTheta;
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXCoordinate(){ return totalX; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYCoordinate(){ return totalY; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnOrientation(){ return Math.toDegrees(robotOrientationRadians) % 360; }

    public double returnGlobalOrientation(){return Math.toDegrees(globalTheta);}

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

    public void reverseLeftEncoder(){
        if(verticalLeftEncoderPositionMultiplier == 1){
            verticalLeftEncoderPositionMultiplier = -1;
        }else{
            verticalLeftEncoderPositionMultiplier = 1;
        }
    }

    public void reverseRightEncoder(){
        if(verticalRightEncoderPositionMultiplier == 1){
            verticalRightEncoderPositionMultiplier = -1;
        }else{
            verticalRightEncoderPositionMultiplier = 1;
        }
    }

    public void reverseNormalEncoder(){
        if(normalEncoderPositionMultiplier == 1){
            normalEncoderPositionMultiplier = -1;
        }else{
            normalEncoderPositionMultiplier = 1;
        }
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
