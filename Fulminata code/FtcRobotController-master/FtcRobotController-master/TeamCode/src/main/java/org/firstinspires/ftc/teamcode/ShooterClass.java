package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp
public class ShooterClass extends LinearOpMode {
    public static double yVelocity (double goalHeight, double robotHeight){
        // Please make sure your units are in meters.
        // This uses a kinematic equation to calculate the velocity of the ring in the "y" or upwards direction
        return Math.sqrt(2 * 9.8 * (goalHeight - robotHeight));
    }

    public static double time (double goalHeight, double robotHeight){
        // Please make sure your units are in meters.
        // This uses a kinematic equation to calculate the time that it takes for the ring to reach its peak,
        // which gives us the time that the ring needs to take in the horizontal direction to meed the goal.

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
    @Override


    public void runOpMode() throws  InterruptedException {
        while (opModeIsActive()){
            double y = yVelocity(0.9, 0);
            double time = time(0.9,0);
            double xVelocity = xVelocity(1.8, time);
            double totalVelocity = totalVelocity(xVelocity, y);

            telemetry.addLine("The total number is..." + totalVelocity);
        }
    }
}
