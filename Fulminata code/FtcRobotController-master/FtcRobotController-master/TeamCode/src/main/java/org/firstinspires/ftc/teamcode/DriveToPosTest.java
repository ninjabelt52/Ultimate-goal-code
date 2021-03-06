package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
@Disabled
public class DriveToPosTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));

        waitForStart();

        while(opModeIsActive()){

                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );

                drive.update();

                Pose2d currentPose = drive.getPoseEstimate();

            if(gamepad1.a){
                Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d(currentPose.getX(), currentPose.getY()))
                        .lineToLinearHeading(new Pose2d(0,0, Math.toRadians(0)))
                        .build();

                drive.followTrajectory(myTrajectory);
            }
        }
    }
}
