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

@Autonomous(name = "E-lem auto BLUE", group = "lemon")
public class LemonAutoBLUE extends LinearOpMode {
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

        drive.setPoseEstimate(new Pose2d(-61, 4, Math.toRadians(180)));

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

        shooter.TurnTurret(.596);


        //lift.move(580);

        switch (analysis) {
            case NONE:
                sleep(14000);
                Trajectory shoot = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,6, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        addTemporalMarker(0, () ->{
                            shooter.startMotor(2150);
                            shooter.TurnTurret(.645);
                        }).
                        build();

                drive.followTrajectory(shoot);


                while(shooter.isNotThere()) {
                    shooter.startMotor(2150);
                    telemetry.addData("shooter Velo", shooter.getVelocity());
                    telemetry.update();
                }

                sleep(500);

                //right powershot
                shooter.TurnTurret(.78);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                //powershot 1&2
                shooter.TurnTurret(.76);
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
                            shooter.startMotor(2150);
                            shooter.TurnTurret(.55);
                        }).
                        build();

                Trajectory shootRing = drive.trajectoryBuilder(new Pose2d(line.end().getX(), line.end().getY(),line.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,6,Math.toRadians(180))).
                        build();

                drive.followTrajectory(line);
                drive.followTrajectory(shootRing);

                while(shooter.isNotThere()){
                    shooter.startMotor(2150);
                    telemetry.addData("shooter Velo",shooter.getVelocity());
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

                shooter.TurnTurret(.75);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                shooter.startMotor(2100);

                while (shooter.isNotThere()){
                    shooter.startMotor(2100);
                    telemetry.addData("velocity", shooter.getVelocity());
                    telemetry.update();
                }
                sleep(500);

                //powershot 1&2
                shooter.TurnTurret(.72);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                shooter.startMotor(2400);
                //tower goal
                while(shooter.isNotThere()){
                    shooter.startMotor(2400);
                    telemetry.addData("shooter Velo",shooter.getVelocity());
                    telemetry.update();
                }
//                sleep(500);
//
//                shooter.TurnTurret(.53);
//                sleep(1000);
//                shooter.shoot();
//                sleep(500);
//                shooter.retract();

                intake1.setPower(1);
                intake2.setPower(1);

                Trajectory pickupRing = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-25, 18, Math.toRadians(180)),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).build();

                Trajectory ring4 = drive.trajectoryBuilder(new Pose2d(pickupRing.end().getX(),pickupRing.end().getY(),pickupRing.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,6, Math.toRadians(180))).
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
                        lineToLinearHeading(new Pose2d(24,16,Math.toRadians(270))).
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

                drive.followTrajectory(moveOff);

                Trajectory homeBase = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(6,10,Math.toRadians(270))).build();

                drive.followTrajectory(homeBase);
                claw.fold();
                sleep(500);
                break;
            case FOUR:
                Trajectory angle = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-21,-15,Math.toRadians(180))).
                        addTemporalMarker(1,() -> {
                            shooter.startMotor(2100);
                            shooter.TurnTurret(.58);
                        }).
                        build();

                Trajectory shootRings = drive.trajectoryBuilder(new Pose2d(angle.end().getX(), angle.end().getY(),angle.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,6,Math.toRadians(180))).
                        addTemporalMarker(0,() -> {
                            shooter.TurnTurret(.55);
                        }).
                        build();

                drive.followTrajectory(angle);
                drive.followTrajectory(shootRings);
                while(shooter.isNotThere()){
                    shooter.startMotor(2100);
                    telemetry.addData("shooter Velo",shooter.getVelocity());
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

                shooter.TurnTurret(.75);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                //powershot 1&2
                shooter.TurnTurret(.72);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                shooter.startMotor(2400);
                //tower goal
                while(shooter.isNotThere()){
                    shooter.startMotor(2400);
                    telemetry.addData("shooter Velo",shooter.getVelocity());
                    telemetry.update();
                }

                intake1.setPower(1);
                intake2.setPower(1);

                Trajectory pickupRings = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-24,16, Math.toRadians(180)),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(12, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        ).build();


                Trajectory rings = drive.trajectoryBuilder(new Pose2d(pickupRings.end().getX(),pickupRings.end().getY(),pickupRings.end().getHeading())).

                        lineToLinearHeading(new Pose2d(-1,6,Math.toRadians(180))).
                        addTemporalMarker(.5, () -> {
                            shooter.TurnTurret(.68);
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

                Trajectory zoneC = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(58,20,Math.toRadians(0))).
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
                intake2.setPower(0);

                Trajectory homeBaseC = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(6,12,Math.toRadians(270))).build();

                drive.followTrajectory(homeBaseC);
                claw.fold();
                sleep(500);
                break;
        }
    }
}
