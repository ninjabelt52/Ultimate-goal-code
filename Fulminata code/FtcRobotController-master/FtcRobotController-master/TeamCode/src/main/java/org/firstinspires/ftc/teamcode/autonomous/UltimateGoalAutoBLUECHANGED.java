package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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

@Autonomous(name = "regular auto Blue side", group = "normal")
public class UltimateGoalAutoBLUE extends LinearOpMode {

    DcMotor intake1;
    OpenCvCamera webcam;
    RingDetermination.RingDeterminationPipeline rings;
    RingDetermination.RingDeterminationPipeline.RingPos analysis;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        wobbleGoal claw = new wobbleGoal(hardwareMap);
        intake1 = hardwareMap.get(DcMotor.class, "intake1");

        drive.setPoseEstimate(new Pose2d(-61,6,Math.toRadians(180)));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        rings = new RingDetermination.RingDeterminationPipeline();
        webcam.setPipeline(rings);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        while(!isStarted()) {
            telemetry.addLine("Initialized update");
            telemetry.addData("Amount of rings", rings.getAnalysis());
            telemetry.addData("Average", rings.Avg());
            telemetry.update();

            drive.update();
        }

        waitForStart();
        analysis = rings.getAnalysis();

        shooter.TurnTurret(.596);


        //lift.move(580);
        //turn turret to start motor
//        while(shooter.getVelocity() < 1000){
//            shooter.startMotor(2380);
//        }

        switch (analysis){
            case NONE:

                Trajectory shoot = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,6, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        addTemporalMarker(0, () ->{
                            shooter.startMotor(1150);
                            shooter.TurnTurret(.645);
                        }).
                        build();

                drive.followTrajectory(shoot);


                while(shooter.isNotThere()) {
                    shooter.startMotor(1150);
                    telemetry.addData("shooter Velo", shooter.getVelocity());
                    telemetry.update();
                }
                sleep(500);

                //right powershot
                shooter.TurnTurret(.77);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                //powershot 1&2
                shooter.TurnTurret(.75);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                shooter.TurnTurret(.73);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                shooter.stopMotor();

                Trajectory wobbleGoal = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY())).
                        lineToLinearHeading(new Pose2d(15,25, 0), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
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
                lift.move(580);

                Trajectory grabWobble = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-37, 20, Math.toRadians(90)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        build();

                drive.followTrajectory(grabWobble);
                lift.move(0);
                sleep(500);

                claw.close();
                sleep(1000);
                lift.move(580);
                claw.fold();

                Trajectory dropWobble2 = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY())).
                        lineToLinearHeading(new Pose2d(9,18, 0)).
                        build();

                drive.followTrajectory(dropWobble2);

                claw.openArm();
                sleep(500);
                lift.move(0);
                claw.openClaw();

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
                Trajectory line = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-21,-12,Math.toRadians(180))).
                        addTemporalMarker(1,() -> {
                            shooter.startMotor(1250);
                            shooter.TurnTurret(.61);
                        }).
                        build();

                Trajectory shootRing = drive.trajectoryBuilder(new Pose2d(line.end().getX(), line.end().getY(),line.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,9,Math.toRadians(180))).
                        build();

                drive.followTrajectory(line);
                drive.followTrajectory(shootRing);

                while(shooter.isNotThere()){
                    shooter.startMotor(1250);
                    telemetry.addData("shooter Velo",shooter.getVelocity());
                    telemetry.update();
                }

                shooter.TurnTurret(.68);
                sleep(1000);
                for(int i = 0; i < 3; i++){
                    shooter.shoot();
                    sleep(500);
                    shooter.retract();
                    sleep(500);
                }

                intake1.setPower(1);

                Trajectory pickupRing = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        forward(24,
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).build();

                Trajectory ring4 = drive.trajectoryBuilder(new Pose2d(pickupRing.end().getX(),pickupRing.end().getY(),pickupRing.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,9, Math.toRadians(180))).
                        addTemporalMarker(.5, () -> {
                            shooter.TurnTurret(.68);
                        }).
                        build();

                drive.followTrajectory(pickupRing);
                drive.followTrajectory(ring4);

                sleep(500);

                shooter.shoot();
                sleep(500);
                shooter.retract();

                Trajectory dropWobble1 = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(42,28,Math.toRadians(180))).
                        addTemporalMarker(1, () -> {
                            shooter.stopMotor();
                            intake1.setPower(0);
                        }).
                        build();

                drive.followTrajectory(dropWobble1);

                claw.openArm();
                sleep(500);
                lift.move(0);
                sleep(500);
                claw.openClaw();
                sleep(500);

                Trajectory moveOff = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(42,32, Math.toRadians(180))).
                        build();

                Trajectory grabWobble2 = drive.trajectoryBuilder(new Pose2d(moveOff.end().getX(), moveOff.end().getY(), moveOff.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-35,20,Math.toRadians(90))).
                        addTemporalMarker(0,() -> {
                            lift.move(580);
                        }).
                        addTemporalMarker(2, () -> {
                            claw.openArm();
                            claw.openArm();
                            lift.move(0);
                        }).
                        build();

                drive.followTrajectory(moveOff);
                drive.followTrajectory(grabWobble2);


                sleep(500);
                claw.close();
                sleep(500);
                lift.move(580);

                Trajectory driveToRelease = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(30, 28, Math.toRadians(180))).
                        build();


                Trajectory releaseWobble2 = drive.trajectoryBuilder(new Pose2d(driveToRelease.end().getX(),driveToRelease.end().getY(),driveToRelease.end().getHeading())).
                        lineToLinearHeading(new Pose2d(36,28,Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        build();

                drive.followTrajectory(driveToRelease);
                drive.followTrajectory(releaseWobble2);

                lift.move(0);
                sleep(100);
                claw.openClaw();

                Trajectory homeBase = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(6,15,Math.toRadians(270))).build();

                drive.followTrajectory(homeBase);
                claw.fold();
                sleep(500);
                break;
                /*Trajectory line = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-21,-14,Math.toRadians(180))).
                        addTemporalMarker(1,() -> {
                            shooter.startMotor(2350);
                            shooter.TurnTurret(.645);
                        }).
                        build();

                Trajectory shootRing = drive.trajectoryBuilder(new Pose2d(line.end().getX(), line.end().getY(),line.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,9,Math.toRadians(180))).
                        build();

                drive.followTrajectory(line);
                drive.followTrajectory(shootRing);

                while(shooter.isNotThere()){
                    shooter.startMotor(2350);
                    telemetry.addData("shooter Velo",shooter.getVelocity());
                    telemetry.update();
                }
                sleep(500);

                //right powershot
                shooter.TurnTurret(.645);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                //powershot 1&2
                shooter.TurnTurret(.6094);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                shooter.startMotor(2550);

                intake1.setPower(1);
                intake2.setPower(1);

                Trajectory pickupRing = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        forward(24,
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).build();

                Trajectory ring4 = drive.trajectoryBuilder(new Pose2d(pickupRing.end().getX(),pickupRing.end().getY(),pickupRing.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,9, Math.toRadians(180))).
                        addTemporalMarker(.5, () -> {
                            shooter.TurnTurret(.55);
                        }).
                        build();

                drive.followTrajectory(pickupRing);
                drive.followTrajectory(ring4);

                shooter.TurnTurret(.55);
                sleep(500);
                while(shooter.isNotThere()){
                    shooter.startMotor(2550);
                    telemetry.addData("shooter Velocity", shooter.getVelocity());
                    telemetry.update();
                }
                sleep(500);

                for(int i = 0; i < 2; i++) {
                    shooter.shoot();
                    sleep(500);
                    shooter.retract();
                    sleep(500);
                }

                Trajectory dropWobble1 = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(42,18,Math.toRadians(180))).
                        addTemporalMarker(1, () -> {
                            shooter.stopMotor();
                            intake1.setPower(0);
                            intake2.setPower(0);
                        }).
                        build();

                drive.followTrajectory(dropWobble1);

                claw.openArm();
                sleep(500);
                lift.move(0);
                sleep(500);
                claw.openClaw();
                sleep(500);

                Trajectory moveOff = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(42,22, Math.toRadians(180))).
                        build();

                Trajectory grabWobble2 = drive.trajectoryBuilder(new Pose2d(moveOff.end().getX(), moveOff.end().getY(), moveOff.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-32.5,18,Math.toRadians(90))).
                        addTemporalMarker(0,() -> {
                            lift.move(580);
                        }).
                        addTemporalMarker(2, () -> {
                            claw.openArm();
                            claw.openArm();
                            lift.move(0);
                        }).
                        build();

                drive.followTrajectory(moveOff);
                drive.followTrajectory(grabWobble2);


                sleep(500);
                claw.close();
                sleep(500);
                lift.move(580);

                Trajectory driveToRelease = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(30, 24, Math.toRadians(180))).
                        build();


                Trajectory releaseWobble2 = drive.trajectoryBuilder(new Pose2d(driveToRelease.end().getX(),driveToRelease.end().getY(),driveToRelease.end().getHeading())).
                        lineToLinearHeading(new Pose2d(36,24,Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        build();

                drive.followTrajectory(driveToRelease);
                drive.followTrajectory(releaseWobble2);

                lift.move(0);
                sleep(100);
                claw.openClaw();

                Trajectory homeBase = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(6,15,Math.toRadians(270))).build();

                drive.followTrajectory(homeBase);
                claw.fold();
                sleep(500);
                break;
                */

            case FOUR:

                Trajectory angle = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-21,-15,Math.toRadians(180))).
                        addTemporalMarker(1,() -> {
                            shooter.startMotor(1250);
                            shooter.TurnTurret(.68);
                        }).
                        build();

                Trajectory shootRings = drive.trajectoryBuilder(new Pose2d(angle.end().getX(), angle.end().getY(),angle.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,9,Math.toRadians(180))).
                        addTemporalMarker(0,() -> {
                            shooter.TurnTurret(.66);
                        }).
                        build();

                drive.followTrajectory(angle);
                drive.followTrajectory(shootRings);
                while(shooter.isNotThere()){
                    shooter.startMotor(1250);
                    telemetry.addData("shooter Velo",shooter.getVelocity());
                    telemetry.update();
                }
                //sleep(500);
                for(int i = 0; i < 3; i++){
                    shooter.shoot();
                    sleep(500);
                    shooter.retract();
                    sleep(500);
                }

                intake1.setPower(1);

                Trajectory pickupRings = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-24,11, Math.toRadians(180)),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(12, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        ).build();


                Trajectory rings = drive.trajectoryBuilder(new Pose2d(pickupRings.end().getX(),pickupRings.end().getY(),pickupRings.end().getHeading())).

                        lineToLinearHeading(new Pose2d(-1,9,Math.toRadians(180))).
                        addTemporalMarker(.5, () -> {
                            shooter.TurnTurret(.66);
                        }).
                        build();

                drive.followTrajectory(pickupRings);
                drive.followTrajectory(rings);


                sleep(1000);

                for(int i = 0; i < 3; i++) {
                    shooter.shoot();
                    sleep(500);
                    shooter.retract();
                    sleep(500);
                }

                shooter.TurnTurret(1);

                Trajectory zoneC = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(58,18,Math.toRadians(0))).
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
                intake1.setPower(0);

                Trajectory wobble2 = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-38,16.5,Math.toRadians(90)),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        addTemporalMarker(0,() -> {
                            lift.move(580);
                        }).
                        addTemporalMarker(3, () -> {
                            claw.openArm();
                            claw.openArm();
                        }).
                        build();

                drive.followTrajectory(wobble2);
                lift.move(0);
                sleep(500);

                claw.close();
                sleep(500);
                lift.move(580);

                Trajectory releaseZoneC = drive.trajectoryBuilder(new Pose2d(wobble2.end().getX(),wobble2.end().getY(),wobble2.end().getHeading())).
                        lineToLinearHeading(new Pose2d(58,16,Math.toRadians(0))).
                        build();

                drive.followTrajectory(releaseZoneC);

                lift.move(0);
                claw.openClaw();

                Trajectory homeBaseC = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(6,12,Math.toRadians(270))).build();

                drive.followTrajectory(homeBaseC);
                claw.fold();
                sleep(500);
                break;
        }



    }
}
