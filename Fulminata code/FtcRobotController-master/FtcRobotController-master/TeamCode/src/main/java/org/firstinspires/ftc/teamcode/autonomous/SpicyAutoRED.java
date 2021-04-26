package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.RingDetermination;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.wobbleGoal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;

@Autonomous(name = "Spicy RED auto", group = "spicy")
public class SpicyAutoRED extends LinearOpMode {
    DcMotor intake1, intake2;
    OpenCvCamera webcam;
    RingDetermination.RingDeterminationPipeline rings;
    RingDetermination.RingDeterminationPipeline.RingPos analysis;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        wobbleGoal claw = new wobbleGoal(hardwareMap);
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        drive.setPoseEstimate(new Pose2d(-61, -36, Math.toRadians(180)));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        rings = new RingDetermination.RingDeterminationPipeline();
        webcam.setPipeline(rings);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        while (!isStarted()) {
            telemetry.addLine("Initialized update");
            telemetry.addData("Amount of rings", rings.getAnalysis());
            telemetry.addData("Average", rings.Avg());
            telemetry.update();

            drive.update();
        }

        waitForStart();

        sleep(5000);
        analysis = rings.getAnalysis();

        shooter.TurnTurret(.596);


        lift.move(580);

        switch (analysis){
            case NONE:
                Trajectory move = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-58, 0, Math.toRadians(180))).
                        build();

                Trajectory shoot = drive.trajectoryBuilder(new Pose2d(move.end().getX(),move.end().getY(),move.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,0, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        addTemporalMarker(0, () ->{
                            shooter.startMotor(2400);
                            shooter.TurnTurret(.76);
                        }).
                        build();

                drive.followTrajectory(move);
                drive.followTrajectory(shoot);


//                while(shooter.isNotThere()) {
//                    shooter.startMotor(2330);
//                    telemetry.addData("shooter Velo", shooter.getVelocity());
//                    telemetry.update();
//                }
//
//                //left powershot
//                shooter.TurnTurret(.445);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();
//                sleep(500);
//
//                //Middle powershot
//                shooter.TurnTurret(.484);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();
//                sleep(500);
//
//                //right powershot
//                shooter.TurnTurret(.55);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();

                while(shooter.isNotThere()){
                    shooter.startMotor(2400);
                    telemetry.addData("velocity", shooter.getVelocity());
                    telemetry.update();
                }
                sleep(1000);

                for(int i = 0 ; i < 3; i++){
                    shooter.shoot();
                    sleep(500);
                    shooter.retract();
                    sleep(500);
                }

                shooter.stopMotor();

                Trajectory wobbleA = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(30,-18, Math.toRadians(90))).
                        build();
                Trajectory wobbleA1 = drive.trajectoryBuilder(new Pose2d(wobbleA.end().getX(),wobbleA.end().getY(),wobbleA.end().getHeading())).
                        lineToLinearHeading(new Pose2d(22,-52, Math.toRadians(90))).
                        addTemporalMarker(1, () -> {
                            claw.openArm();
                        }).
                build();

                drive.followTrajectory(wobbleA);
                sleep(5000);
                drive.followTrajectory(wobbleA1);

                claw.openArm();
                sleep(500);
                lift.move(0);
                sleep(500);
                claw.openClaw();
                sleep(500);
                lift.move(580);
                sleep(500);
                claw.fold();

                Trajectory park1 = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(48,-18, Math.toRadians(90))).
                        build();
                Trajectory park2 = drive.trajectoryBuilder(new Pose2d(park1.end().getX(),park1.end().getY(),park1.end().getHeading())).
                        lineToLinearHeading(new Pose2d(6,0, Math.toRadians(0))).
                        build();

                drive.followTrajectory(park1);
                drive.followTrajectory(park2);

                lift.move(0);
                sleep(500);
                break;
                case ONE:
                    Trajectory moveB = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                            lineToLinearHeading(new Pose2d(-58, 0, Math.toRadians(180))).
                            build();

                    Trajectory shootB = drive.trajectoryBuilder(new Pose2d(moveB.end().getX(),moveB.end().getY(),moveB.end().getHeading())).
                            lineToLinearHeading(new Pose2d(-1,0, Math.toRadians(180)), new MinVelocityConstraint(
                                            Arrays.asList(
                                                    new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                    new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                            )
                                    ),
                                    new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                            addTemporalMarker(0, () ->{
                                shooter.startMotor(2300);
                                shooter.TurnTurret(.76);
                            }).
                            build();

                    drive.followTrajectory(moveB);
                    drive.followTrajectory(shootB);


//                while(shooter.isNotThere()) {
//                    shooter.startMotor(2330);
//                    telemetry.addData("shooter Velo", shooter.getVelocity());
//                    telemetry.update();
//                }
//
//                //left powershot
//                shooter.TurnTurret(.445);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();
//                sleep(500);
//
//                //Middle powershot
//                shooter.TurnTurret(.484);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();
//                sleep(500);
//
//                //right powershot
//                shooter.TurnTurret(.55);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();

                    while(shooter.isNotThere()){
                        shooter.startMotor(2300);
                        telemetry.addData("velocity", shooter.getVelocity());
                        telemetry.update();
                    }
                    sleep(1000);

                    for(int i = 0 ; i < 3; i++){
                        shooter.shoot();
                        sleep(500);
                        shooter.retract();
                        sleep(500);
                    }


                    shooter.stopMotor();

                    Trajectory wobbleB = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                            lineToLinearHeading(new Pose2d(36,-20,Math.toRadians(180))).
                            addTemporalMarker(1, () -> {
                                claw.openArm();
                            }).
                            build();

                    drive.followTrajectory(wobbleB);

                    lift.move(0);
                    sleep(500);
                    claw.openClaw();
                    sleep(500);
                    lift.move(580);
                    sleep(500);
                    claw.fold();

                    Trajectory moveOffB = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                            lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(180))).
                            build();

                    Trajectory parkB = drive.trajectoryBuilder(new Pose2d(moveOffB.end().getX(), moveOffB.end().getY(), moveOffB.end().getHeading())).
                            lineToLinearHeading(new Pose2d(6, 0, Math.toRadians(0))).
                            build();

                    drive.followTrajectory(moveOffB);
                    drive.followTrajectory(parkB);

                    lift.move(0);
                    sleep(500);
                    break;
            case FOUR:
                shooter.startMotor(2300);
                while (shooter.isNotThere()){
                    shooter.startMotor(2300);
                    telemetry.addData("velocity", shooter.getVelocity());
                    telemetry.update();
                }

                Trajectory moveC = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-58, 0, Math.toRadians(180))).
                        build();

                Trajectory shootC = drive.trajectoryBuilder(new Pose2d(moveC.end().getX(),moveC.end().getY(),moveC.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,0, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        addTemporalMarker(0, () ->{
                            shooter.startMotor(2300);
                            shooter.TurnTurret(.7503);
                        }).
                        build();

                drive.followTrajectory(moveC);
                drive.followTrajectory(shootC);


//                while(shooter.isNotThere()) {
//                    shooter.startMotor(2330);
//                    telemetry.addData("shooter Velo", shooter.getVelocity());
//                    telemetry.update();
//                }
//
//                //left powershot
//                shooter.TurnTurret(.445);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();
//                sleep(500);
//
//                //Middle powershot
//                shooter.TurnTurret(.484);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();
//                sleep(500);
//
//                //right powershot
//                shooter.TurnTurret(.55);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();

                while(shooter.isNotThere()){
                    shooter.startMotor(2300);
                    telemetry.addData("velocity", shooter.getVelocity());
                    telemetry.update();
                }
                sleep(1000);

                for(int i = 0 ; i < 3; i++){
                    shooter.shoot();
                    sleep(500);
                    shooter.retract();
                    sleep(500);
                }

                shooter.stopMotor();

                Trajectory wobbleC = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(60,-42,Math.toRadians(180))).
                        addTemporalMarker(1, () -> {
                            claw.openArm();
                        }).
                        build();

                drive.followTrajectory(wobbleC);

                lift.move(0);
                sleep(500);
                claw.openClaw();
                sleep(500);
                lift.move(580);
                sleep(500);
                claw.fold();

                Trajectory ParkC1 = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(60, 0, Math.toRadians(180))).
                        build();

                Trajectory parkC = drive.trajectoryBuilder(new Pose2d(ParkC1.end().getX(),ParkC1.end().getY(),ParkC1.end().getHeading())).
                        lineToLinearHeading(new Pose2d(6,0, Math.toRadians(0))).
                        build();

                drive.followTrajectory(ParkC1);
                drive.followTrajectory(parkC);

                lift.move(0);
                sleep(500);
                break;
        }
    }
}
