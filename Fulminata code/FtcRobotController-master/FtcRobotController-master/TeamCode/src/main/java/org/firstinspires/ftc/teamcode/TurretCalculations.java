package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class TurretCalculations implements Runnable {
    public static final int ROTATION_DEGREES = 180;
    public Servo turret;
    public boolean loop = true;
    private double calculation = 0;
    private double leg1 = 0;
    private double leg2 = 0;
    public static double angleCorrection = 10;

    public CameraView vision;
    //public ShooterThread shooter;
    public Thread shooterThread;
    public Thread visionThread;
    public String status = "";

    private final Vector2d TOP_GOAL_POS = new Vector2d((double)72 * 25.4,(double)36 * 25.4);


    FtcDashboard dash;
    Telemetry telemetry;

    public TurretCalculations(HardwareMap hardwaremap){
        dash = FtcDashboard.getInstance();
        telemetry = dash.getTelemetry();
        status = "in constructor";
        vision = new CameraView(hardwaremap);
        //shooter = new ShooterThread(hardwaremap);

        visionThread = new Thread(vision);
        //shooterThread = new Thread(shooter);

        turret = hardwaremap.get(Servo.class, "turret");
        turret.scaleRange(.25,.98);
        visionThread.start();
        status = "initialized";
        FtcDashboard.getInstance().startCameraStream(vision.vuforia, 0);

    }

    public static double yVelocity (double goalHeight, double robotHeight){
        // Please make sure your units are in meters.
        // This uses a kinematic equation to calculate the velocity of the ring in the "y" or upwards direction
        return Math.sqrt(2 * 9.8 * (goalHeight - robotHeight));
    }

    public static double time (double goalHeight, double robotHeight){
        // Please make sure your units are in meters.
        // This uses a kinematic equation to calculate the time that it takes for the ring to reach its peak, which gives us the time that the ring needs to take in the horizontal direction to meed the goal.

        return Math.sqrt((goalHeight - robotHeight)/(9.8/2));
    }

    public static double xVelocity (double distance, double time){
        // Please make sure your units are in meters.
        // This calculates the velocity of the ring in the "x" or horizontal direction.

        return (distance/time);
    }

    public static double totalVelocity (double x, double y){
        // This calculates the total velocity of the ring, using the pythagorean theorem, a^2 + b^2 = c^2, or square root of (a^2 + b^2) = c
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public void run(){
        while (loop){
            turret.scaleRange(.25, .98);
            //aiming code
            //getting the lengths of legs of a triangle
            leg1 = Math.abs(TOP_GOAL_POS.getX() - vision.returnXCoordinate());
            leg2 = TOP_GOAL_POS.getY() - vision.returnYCoordinate();

            //calculating the angle from the robot to the tower goal, minus the rotation of the robot
            calculation = Math.toDegrees(Math.asin(leg1/Math.hypot(leg1, leg2))) + vision.returnRotation() + angleCorrection;
            //the servo needs a decimal value between 0 and 1, so the degrees divided by the total amount of degrees
            // of rotation that the robot is able to move gives us this number
            //zero opposite
            turret.setPosition(Math.abs(calculation/ROTATION_DEGREES));
        }
        shooterThread.interrupt();
    }

    public String visionData(){
        return "Y coordinate: " + vision.returnYCoordinate()
                + "\nX coordinate: " + vision.returnXCoordinate()
                + "\nrotation: " + vision.returnRotation()
                + "\nstatus: " + vision.getStatus();
    }

    public String getStatus(){
        return status;
    }

    public void setAngleCorrection(int correction){
        angleCorrection = correction;
    }

    public double getAngleCorrection(){
        return angleCorrection;
    }

    public double getSetPosition(){
        return Math.abs(calculation/ROTATION_DEGREES);
    }

    public double getCalculation(){
        return calculation;
    }
}
