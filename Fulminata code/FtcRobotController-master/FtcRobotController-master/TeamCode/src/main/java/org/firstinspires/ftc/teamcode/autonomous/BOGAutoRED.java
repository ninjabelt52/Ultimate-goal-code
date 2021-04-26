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

@Autonomous(name = "Red BOG auto", group = "BOG")
public class BOGAutoRED extends LinearOpMode {
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
        analysis = rings.getAnalysis();

        shooter.TurnTurret(.55);
        switch (analysis){
            case NONE:
                sleep(10000);

                Trajectory shoot = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,-36, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        addTemporalMarker(0, () ->{
                            shooter.startMotor(2450);
                            shooter.TurnTurret(.67);
                        }).
                        build();

                drive.followTrajectory(shoot);

                while(shooter.isNotThere()){
                    shooter.startMotor(2450);
                    telemetry.addData("shooter velo", shooter.getVelocity());
                    telemetry.update();
                }
                sleep(100);

                for(int i = 0; i < 3; i++){
                    shooter.shoot();
                    sleep(500);
                    shooter.retract();
                    sleep(500);
                }

                shooter.stopMotor();

                Trajectory wobbleGoal = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(18,-46, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        build();

                drive.followTrajectory(wobbleGoal);

                claw.openArm();
                sleep(500);
                lift.move(0);
                sleep(500);
                claw.openClaw();
                sleep(500);

                Trajectory park = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        strafeRight(18).
                        addTemporalMarker(1, () ->{
                            claw.fold();
                        }).
                        build();

                drive.followTrajectory(park);
                claw.fold();
                sleep(500);
                break;
            case ONE:
                sleep(10000);

                Trajectory lineUpB = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-21, -48, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        build();

                Trajectory shootB = drive.trajectoryBuilder(new Pose2d(lineUpB.end().getX(), lineUpB.end().getY(), lineUpB.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,-36, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        addTemporalMarker(0, () ->{
                            shooter.startMotor(2450);
                            shooter.TurnTurret(.66);
                        }).
                        build();

                drive.followTrajectory(lineUpB);
                drive.followTrajectory(shootB);

                while(shooter.isNotThere()){
                    shooter.startMotor(2450);
                    telemetry.addData("shooter velo", shooter.getVelocity());
                    telemetry.update();
                }
                sleep(500);

                for(int i = 0; i < 3; i++){
                    shooter.shoot();
                    sleep(500);
                    shooter.retract();
                    sleep(500);
                }

                intake1.setPower(1);
                intake2.setPower(1);

                Trajectory grabRing = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-25, -24, Math.toRadians(180))).
                        build();

                Trajectory shootRing = drive.trajectoryBuilder(new Pose2d(grabRing.end().getX(), grabRing.end().getY(), grabRing.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,-36, Math.toRadians(180))).
                        build();

                drive.followTrajectory(grabRing);
                drive.followTrajectory(shootRing);

                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                shooter.stopMotor();
                intake1.setPower(0);
                intake2.setPower(0);

                Trajectory wobbleGoalB = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(36,-12, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        build();

                Trajectory moveOffB = drive.trajectoryBuilder(new Pose2d(wobbleGoalB.end().getX(), wobbleGoalB.end().getY(), wobbleGoalB.end().getHeading())).
                        lineToLinearHeading(new Pose2d(36,-6, Math.toRadians(180))).
                        build();

                drive.followTrajectory(wobbleGoalB);

                claw.openArm();
                sleep(500);
                lift.move(0);
                sleep(500);
                claw.openClaw();
                sleep(500);
                lift.move(580);
                sleep(500);
                claw.fold();

                drive.followTrajectory(moveOffB);


                Trajectory parkB = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(6,-6, Math.toRadians(0))).
                        addTemporalMarker(1, () ->{
                            claw.fold();
                        }).
                        build();

                drive.followTrajectory(parkB);
                lift.move(0);
                sleep(500);
                break;

            case FOUR:
                sleep(5000);
                shooter.startMotor(2450);
                while (shooter.isNotThere()){
                    shooter.startMotor(2450);
                    telemetry.addData("velocity", shooter.getVelocity());
                    telemetry.update();
                }

                Trajectory lineUpC = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-21, -48, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        build();

                Trajectory shootC = drive.trajectoryBuilder(new Pose2d(lineUpC.end().getX(), lineUpC.end().getY(), lineUpC.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,-36, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        addTemporalMarker(0, () ->{
                            shooter.startMotor(2450);
                            shooter.TurnTurret(.66);
                        }).
                        build();

                drive.followTrajectory(lineUpC);
                drive.followTrajectory(shootC);

                while(shooter.isNotThere()){
                    shooter.startMotor(2450);
                    telemetry.addData("shooter velo", shooter.getVelocity());
                    telemetry.update();
                }
                sleep(500);

                for(int i = 0; i < 3; i++){
                    shooter.shoot();
                    sleep(500);
                    shooter.retract();
                    sleep(500);
                }

                intake1.setPower(1);
                intake2.setPower(1);

                Trajectory grabRingC = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-25, -24, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        build();

                Trajectory shootRingC = drive.trajectoryBuilder(new Pose2d(grabRingC.end().getX(), grabRingC.end().getY(), grabRingC.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,-36, Math.toRadians(180))).
                        addTemporalMarker(0, () -> {
                            shooter.TurnTurret(.68);
                        }).
                        build();

                drive.followTrajectory(grabRingC);
                drive.followTrajectory(shootRingC);

                for(int i = 0; i < 3; i++){
                    shooter.shoot();
                    sleep(500);
                    shooter.retract();
                    sleep(500);
                }

                shooter.stopMotor();
                intake1.setPower(0);
                intake2.setPower(0);

                Trajectory wobbleGoalC = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(60,-46, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        build();

                Trajectory moveOffC = drive.trajectoryBuilder(new Pose2d(wobbleGoalC.end().getX(), wobbleGoalC.end().getY(), wobbleGoalC.end().getHeading())).
                        lineToLinearHeading(new Pose2d(60,-12, Math.toRadians(180))).
                        build();

                drive.followTrajectory(wobbleGoalC);

                claw.openArm();
                sleep(500);
                lift.move(0);
                sleep(500);
                claw.openClaw();
                sleep(500);
                lift.move(580);
                sleep(500);
                claw.fold();

                drive.followTrajectory(moveOffC);


                Trajectory parkC = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(6,-12, Math.toRadians(0))).
                        addTemporalMarker(1, () ->{
                            claw.fold();
                        }).
                        build();

                drive.followTrajectory(parkC);
                lift.move(0);
                sleep(500);
                break;
        }
    }
}
