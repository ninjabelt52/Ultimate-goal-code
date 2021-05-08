package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TurretCalculations implements Runnable {
    private final int ROTATION_DEGREES = 200;
    private Servo turret;
    private final Vector2d TOP_GOAL_POS = new Vector2d(0,0);
    public static SampleMecanumDrive drive;
    public boolean loop = true;
    private double calculation = 0;
    private double leg1 = 0;
    private double leg2 = 0;

    public TurretCalculations(HardwareMap hardwaremap){
        drive = new SampleMecanumDrive(hardwaremap);

        turret = hardwaremap.get(Servo.class, "turret");
    }

    public void setStartPose(Pose2d startPose){
        drive.setPoseEstimate(startPose);
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
            leg1 = TOP_GOAL_POS.getX() - drive.getPoseEstimate().getX();
            leg2 = TOP_GOAL_POS.getY() - drive.getPoseEstimate().getY();
            calculation = Math.asin(leg2/Math.hypot(leg1, leg2));
            turret.setPosition(calculation/ROTATION_DEGREES);
        }
    }
}
