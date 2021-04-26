package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name = "Spicy BLUE auto", group = "spicy")
public class SpicyAutoBLUE extends LinearOpMode {
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

        drive.setPoseEstimate(new Pose2d(-61, 6, Math.toRadians(180)));

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
        analysis = rings.getAnalysis();

        lift.move(580);

        switch (analysis){
            case NONE:
                Trajectory shoot = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,-18, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        addTemporalMarker(0, () ->{
                            shooter.startMotor(2400);
                            shooter.TurnTurret(.61);
                        }).
                        build();

                drive.followTrajectory(shoot);


                while(shooter.isNotThere()) {
                    shooter.startMotor(2400);
                    telemetry.addData("shooter Velo", shooter.getVelocity());
                    telemetry.update();
                }

                for(int i = 0; i < 3; i++){
                    shooter.shoot();
                    sleep(500);
                    shooter.retract();
                    sleep(500);
                }

//                //right powershot
//                shooter.TurnTurret(.59238);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();
//                sleep(500);
//
//                //center powershot
//                shooter.TurnTurret(.55214);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();
//                sleep(500);
//
//                //left powershot
//                shooter.TurnTurret(.53101);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();

                shooter.stopMotor();

                Trajectory move = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(48, 0, Math.toRadians(90))).
                        addTemporalMarker(0,() -> {
                            claw.openArm();
                        }).
                        build();

                Trajectory wobbleGoal = drive.trajectoryBuilder(new Pose2d(move.end().getX(),move.end().getY(),move.end().getHeading())).
                        lineToLinearHeading(new Pose2d(24,33, Math.toRadians(90)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        build();

                drive.followTrajectory(move);
                sleep(5000);
                drive.followTrajectory(wobbleGoal);

                lift.move(0);
                sleep(500);
                claw.openClaw();
                sleep(500);
                lift.move(580);
                sleep(500);
                claw.fold();

                Trajectory move2 = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(48, 0, Math.toRadians(90))).
                        build();

                Trajectory parkA = drive.trajectoryBuilder(new Pose2d(move2.end().getX(),move2.end().getY(),move2.end().getHeading())).
                        lineToLinearHeading(new Pose2d(6,-24, Math.toRadians(0))).
                        build();

                drive.followTrajectory(move2);
                drive.followTrajectory(parkA);
                lift.move(0);
                sleep(500);
                break;

            case ONE:
                Trajectory shootB = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,-18, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        addTemporalMarker(0, () ->{
                            shooter.startMotor(2400);
                            shooter.TurnTurret(.61);
                        }).
                        build();

                drive.followTrajectory(shootB);


                while(shooter.isNotThere()) {
                    shooter.startMotor(2400);
                    telemetry.addData("shooter Velo", shooter.getVelocity());
                    telemetry.update();
                }

                for(int i = 0; i < 3; i++){
                    shooter.shoot();
                    sleep(500);
                    shooter.retract();
                    sleep(500);
                }

//                //right powershot
//                shooter.TurnTurret(.57238);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();
//                sleep(500);
//
//                //center powershot
//                shooter.TurnTurret(.55114);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();
//                sleep(500);
//
//                //left powershot
//                shooter.TurnTurret(.53001);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();

                Trajectory dropWobble1 = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(42,-6,Math.toRadians(0))).
                        addTemporalMarker(1, () -> {
                            shooter.stopMotor();
                        }).
                        build();

                drive.followTrajectory(dropWobble1);

                claw.openArm();
                sleep(500);
                lift.move(0);
                sleep(500);
                claw.openClaw();
                sleep(500);
                lift.move(580);
                sleep(500);
                claw.fold();

                Trajectory parkB = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(6, -12, Math.toRadians(0))).
                        build();

                drive.followTrajectory(parkB);

                lift.move(0);
                sleep(500);
                break;

            case FOUR:
                Trajectory shootC = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,-18, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        addTemporalMarker(0, () ->{
                            shooter.startMotor(2400);
                            shooter.TurnTurret(.61);
                        }).
                        build();

                drive.followTrajectory(shootC);


                while(shooter.isNotThere()) {
                    shooter.startMotor(2400);
                    telemetry.addData("shooter Velo", shooter.getVelocity());
                    telemetry.update();
                }

                for(int i = 0; i < 3; i++){
                    shooter.shoot();
                    sleep(500);
                    shooter.retract();
                    sleep(500);
                }

//                //right powershot
//                shooter.TurnTurret(.59238);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();
//                sleep(500);
//
//                //center powershot
//                shooter.TurnTurret(.55214);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();
//                sleep(500);
//
//                //left powershot
//                shooter.TurnTurret(.53101);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();

                shooter.stopMotor();

                Trajectory zoneC = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(62,16,Math.toRadians(0))).
                        addTemporalMarker(1,() -> {
                            claw.openArm();
                            shooter.stopMotor();
                            //intake.setPower(0);
                        }).
                        build();
                drive.followTrajectory(zoneC);

                lift.move(0);
                sleep(500);
                claw.openClaw();
                sleep(500);
                lift.move(580);
                sleep(500);
                claw.fold();

                Trajectory moveC = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(62,0, Math.toRadians(0))).
                        build();

                Trajectory ParkC = drive.trajectoryBuilder(new Pose2d(moveC.end().getX(),moveC.end().getY(),moveC.end().getHeading())).
                        lineToLinearHeading(new Pose2d(6, -18, Math.toRadians(0))).
                        build();

                drive.followTrajectory(moveC);
                drive.followTrajectory(ParkC);
                lift.move(0);
                sleep(500);
                break;
        }
    }
}
