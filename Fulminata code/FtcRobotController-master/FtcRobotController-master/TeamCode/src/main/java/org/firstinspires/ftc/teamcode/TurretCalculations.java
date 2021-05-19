package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Config
public class TurretCalculations implements Runnable {
    public static final int ROTATION_DEGREES = 165;
    public Servo turret;
    public boolean loop = true;
    private boolean toggle = false;
    private boolean toggle2 = false;
    private boolean track = false;

    private Target currentTar;

    private double calculation = 0;
    private double leg1 = 0;
    private double leg2 = 0;
    private double turretReduction = .04;
    public double pos = 1;
    public static double angleCorrection = 0;

    public CameraView vision;
    //public ShooterThread shooter;
    public Thread shooterThread;
    public Thread visionThread;
    public String status = "";

    private ArrayList<Vector2d> targets = new ArrayList<Vector2d>();

    private Vector2d targetPos;

    Gamepad gamepad2;

    Telemetry telemetry;

    public TurretCalculations(HardwareMap hardwaremap, Gamepad gamepad, Telemetry telemetry, Target tar){
        this.telemetry = telemetry;
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
        gamepad2 = gamepad;

        setTarget(tar);
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

    public enum Target{
        BLUE_TOWER_GOAL,
        RED_TOWER_GOAL,
        BLUE_POWERSHOT1,
        BLUE_POWERSHOT2,
        BLUE_POWERSHOT3,
        RED_POWERSHOT1,
        RED_POWERSHOT2,
        RED_POWERSHOT3
    }

    public void run(){
        while (loop){
            turret.scaleRange(.25, .98);
            status = vision.getStatus();

            if(gamepad2.left_trigger > 0 && gamepad2.dpad_left){
                if(!toggle){
                    angleCorrection -= 1;
                    telemetry.speak(angleCorrection + "degrees");
                    telemetry.update();
                    toggle = true;
                }
            }else if(gamepad2.dpad_right && gamepad2.left_trigger > 0){
                if(!toggle){
                    angleCorrection += 1;
                    telemetry.speak(angleCorrection + "degrees");
                    telemetry.update();
                    toggle = true;
                }
            }else if(gamepad2.dpad_right){
                if(!toggle){
                    angleCorrection = angleCorrection - angleCorrection % 5;
                    angleCorrection += 5;
                    telemetry.speak(angleCorrection + "degrees");
                    telemetry.update();
                    toggle = true;
                }
            }else if(gamepad2.dpad_left){
                if(!toggle){
                    angleCorrection -= 5;
                    if(angleCorrection < 0) {
                        angleCorrection = angleCorrection - angleCorrection % 5;
                    }else{
                        angleCorrection += 4;
                        angleCorrection = angleCorrection - angleCorrection % 5;
                    }
                    telemetry.speak(angleCorrection + "degrees");
                    telemetry.update();
                    toggle = true;
                }
            }else{
                toggle = false;
            }

            if (gamepad2.x) {
                if (!toggle2) {
                    track = !track;
                    toggle2 = true;
                }
            }else{
                toggle2 = false;
            }

            if(track) {
                vision.setPlaySound(true);
                //aiming code
                //getting the lengths of legs of a triangle
                leg1 = Math.abs(targetPos.getX() - vision.returnXCoordinate());
                leg2 = targetPos.getY() - vision.returnYCoordinate();

                //calculating the angle from the robot to the tower goal, minus the rotation of the robot
                //do arccosin in order to account for the first and second quadrants(or the front two, not
                //sure what they are called)
                calculation = Math.toDegrees(Math.acos(leg2/Math.hypot(leg1, leg2))) + vision.returnRotation() + angleCorrection;

                //the servo needs a decimal value between 0 and 1, so the degrees divided by the total amount of degrees
                // of rotation that the robot is able to move gives us this number
                turret.setPosition(Math.abs(calculation / ROTATION_DEGREES));
                pos = turret.getPosition();
            }else{
                vision.setPlaySound(false);
                if(gamepad2.right_stick_x != 0){
                    pos += gamepad2.right_stick_x * turretReduction;
                }else if(gamepad2.y){
                    pos = 0.035;
                }

                if(gamepad2.right_bumper){
                    turretReduction = .005;
                }else{
                    turretReduction = .04;
                }

                pos = Range.clip(pos,0, 1);

                turret.setPosition(pos);
            }
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

    public boolean isTracking(){
        return track;
    }

    public void setTarget(Target tar){
        currentTar = tar;
        switch (tar){
            case BLUE_TOWER_GOAL:
                targetPos = new Vector2d((double) 72 * 25.4, (double) 36 * 25.4);
                break;
            case RED_TOWER_GOAL:
                targetPos = new Vector2d((double) 72 * 25.4, (double) -36 * 25.4);
                break;
            case BLUE_POWERSHOT1:
                targetPos = new Vector2d((double) 72 * 25.4, 18.5 * 25.4);
                break;
            case BLUE_POWERSHOT2:
                targetPos = new Vector2d((double) 72 * 25.4, (double) 11 * 25.4);
                break;
            case BLUE_POWERSHOT3:
                targetPos = new Vector2d((double) 72 * 25.4, 3.5 * 25.4);
                break;
            case RED_POWERSHOT1:
                targetPos = new Vector2d((double) 72 * 25.4, -18.5 * 25.4);
                break;
            case RED_POWERSHOT2:
                targetPos = new Vector2d((double) 72 * 25.4, (double) -11 * 25.4);
                break;
            case RED_POWERSHOT3:
                targetPos = new Vector2d((double) 72 * 25.4, -3.5 * 25.4);
                break;
        }
    }

    public Target getCurrentTar(){
        return currentTar;
    }
}
