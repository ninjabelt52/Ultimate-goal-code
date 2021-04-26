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

@Autonomous(name = "E-lem auto RED", group = "lemon")
public class LemonAutoRED extends LinearOpMode {
    DcMotor intake1, intake2;
    OpenCvCamera webcam;
    RingDetermination.RingDeterminationSpecial rings;
    RingDetermination.RingDeterminationSpecial.RingPos analysis;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        wobbleGoal claw = new wobbleGoal(hardwareMap);
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        drive.setPoseEstimate(new Pose2d(-61, -24, Math.toRadians(180)));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        rings = new RingDetermination.RingDeterminationSpecial();
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

        switch (analysis){
            case NONE:
                sleep(12000);
                Trajectory shoot = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,-36, Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        addTemporalMarker(0, () ->{
                            shooter.startMotor(2150);
                            shooter.TurnTurret(.55);
                        }).
                        build();

                drive.followTrajectory(shoot);

                /**Not going to use this powershot code for now, it will take some time to get this tuned
                 * UPDATE: now attempting to use this code
                 * **/
                while(shooter.isNotThere()) {
                    shooter.startMotor(2150);
                    telemetry.addData("shooter Velo", shooter.getVelocity());
                    telemetry.update();
                }

                //left powershot
                shooter.TurnTurret(.6);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                //powershot 1&2
                shooter.TurnTurret(.625);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                shooter.TurnTurret(.635);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();

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
                Trajectory line = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-21,0,Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        addTemporalMarker(1,() -> {
                            shooter.startMotor(2050);
                            shooter.TurnTurret(.57);
                        }).
                        build();

                Trajectory shootRing = drive.trajectoryBuilder(new Pose2d(line.end().getX(), line.end().getY(),line.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,-36,Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        build();

                drive.followTrajectory(line);
                drive.followTrajectory(shootRing);

                while(shooter.isNotThere()) {
                    shooter.startMotor(2050);
                    telemetry.addData("shooter Velo", shooter.getVelocity());
                    telemetry.update();
                }
                sleep(500);

                //left powershot
                shooter.TurnTurret(.59);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                //powershot 1&2
                shooter.TurnTurret(.605);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                shooter.TurnTurret(.62);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();

                shooter.startMotor(2350);
                while (shooter.isNotThere()){
                    shooter.startMotor(2350);
                    telemetry.addData("shooter velocity", shooter.getVelocity());
                    telemetry.update();
                }

                intake1.setPower(1);
                intake2.setPower(1);

                Trajectory pickupRing = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-25,-36, Math.toRadians(180)),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).build();

                Trajectory ring4 = drive.trajectoryBuilder(new Pose2d(pickupRing.end().getX(),pickupRing.end().getY(),pickupRing.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,-36, Math.toRadians(180))).
                        addTemporalMarker(.5, () -> {
                            shooter.TurnTurret(.67);
                        }).
                        build();

                drive.followTrajectory(pickupRing);
                drive.followTrajectory(ring4);

                sleep(500);

                shooter.shoot();
                sleep(500);
                shooter.retract();

                Trajectory dropWobble1 = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(42,-20,Math.toRadians(180))).
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
                        lineToLinearHeading(new Pose2d(42,0, Math.toRadians(180))).
                        build();

                drive.followTrajectory(moveOff);

                Trajectory homeBase = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(6,-16,Math.toRadians(0))).build();

                drive.followTrajectory(homeBase);
                claw.fold();
                sleep(500);
                break;
            case FOUR:
                Trajectory divert = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(-21,0,Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        addTemporalMarker(1,() -> {
                            shooter.startMotor(2000);
                            shooter.TurnTurret(.57);
                        }).
                        build();

                Trajectory shootRings = drive.trajectoryBuilder(new Pose2d(divert.end().getX(), divert.end().getY(),divert.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,-36,Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        build();

                drive.followTrajectory(divert);
                drive.followTrajectory(shootRings);

                while(shooter.isNotThere()){
                    shooter.startMotor(2000);
                    telemetry.addData("shooter Velo",shooter.getVelocity());
                    telemetry.update();
                }
                sleep(500);

                //left powershot
                shooter.TurnTurret(.59);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                //powershot 1&2
                shooter.TurnTurret(.615);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();
                sleep(500);

                shooter.TurnTurret(.63);
                sleep(1000);
                shooter.shoot();
                sleep(500);
                shooter.retract();

                shooter.startMotor(2300);
                while (shooter.isNotThere()){
                    shooter.startMotor(2300);
                    telemetry.addData("shooter velocity", shooter.getVelocity());
                    telemetry.update();
                }

                intake1.setPower(1);
                intake2.setPower(1);

                Trajectory pickupRings = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),drive.getPoseEstimate().getHeading())).
                        forward(20,
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).build();

                Trajectory backLine = drive.trajectoryBuilder(new Pose2d(pickupRings.end().getX(),pickupRings.end().getY(),pickupRings.end().getHeading())).
                        lineToLinearHeading(new Pose2d(-1,-36, Math.toRadians(180))).
                        addTemporalMarker(.5, () -> {
                            shooter.TurnTurret(.67);
                        }).
                        build();

                drive.followTrajectory(pickupRings);
                drive.followTrajectory(backLine);

                for(int i = 0; i < 3; i++){
                    shooter.shoot();
                    sleep(500);
                    shooter.retract();
                    sleep(500);
                }


                Trajectory zoneC = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(48, -40, Math.toRadians(180))).
                        addTemporalMarker(0, () -> {
                            shooter.TurnTurret(1);
                        }).
                        build();

                Trajectory zoneCSlow = drive.trajectoryBuilder(new Pose2d(zoneC.end().getX(),zoneC.end().getY(),zoneC.end().getHeading())).
                        lineToLinearHeading(new Pose2d(60,-40,Math.toRadians(180)), new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)).
                        addTemporalMarker(1,() -> {
                            claw.openArm();
                            shooter.stopMotor();
                            //intake.setPower(0);
                        }).
                        build();

                drive.followTrajectory(zoneC);
                drive.followTrajectory(zoneCSlow);



                lift.move(0);
                sleep(500);
                claw.openClaw();
                sleep(500);
                intake1.setPower(0);
                intake2.setPower(0);
                lift.move(580);
                sleep(500);
                claw.fold();
                sleep(500);

                Trajectory homeBaseC = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())).
                        lineToLinearHeading(new Pose2d(6,-24,Math.toRadians(0))).build();

                drive.followTrajectory(homeBaseC);
                lift.move(0);
                sleep(500);
                break;
        }
    }
}
