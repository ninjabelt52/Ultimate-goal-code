package org.firstinspires.ftc.teamcode;

import android.text.Html;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp (name = "TeleOpMode RED")
public class UltimateGoalTeleOpRED extends LinearOpMode {
    public TurretCalculations turret;

    @Override
    public void runOpMode() throws InterruptedException{
//        FtcDashboard dash = FtcDashboard.getInstance();
//        Telemetry telemetry = dash.getTelemetry();
        DcMotor lift, intakeMotor1,intakeMotor2;
        Servo liftRotateServo;
        Servo clawServo;
        //Servo turret;
        //RingCounter counter;
        //DcMotorEx shooter;
        //Servo kicker;
        //PIDFController pid = new PIDFController(1,1,.06,1);


        lift = hardwareMap.get(DcMotor.class, "lift");
        //turret = hardwareMap.get(Servo.class, "turret");
        //shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        //kicker = hardwareMap.get(Servo.class, "kicker");
        //intakeMotor1 = hardwareMap.get(DcMotor.class, "intake1");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake1");
        liftRotateServo = hardwareMap.get(Servo.class, "rotate");
        clawServo = hardwareMap.get(Servo.class, "claw");
        //counter = new RingCounter(hardwareMap);
        //Lights blinkin = new Lights(hardwareMap, gamepad1, kicker);
        //Thread lights = new Thread(blinkin);


        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
        DriveThread driveThread = new DriveThread(hardwareMap, gamepad1);
        ShooterThread shooterThread = new ShooterThread(hardwareMap, gamepad2, telemetry);
        Shoot shoot = new Shoot(hardwareMap, gamepad1);
        turret = new TurretCalculations(hardwareMap, gamepad2, telemetry, TurretCalculations.Target.RED_TOWER_GOAL);
        Thread turretThread = new Thread(turret);
        KnockDownArm arm = new KnockDownArm(hardwareMap, gamepad1);
        Thread armThread = new Thread(arm);
        Thread shooter = new Thread(shooterThread);
        Thread drive = new Thread(driveThread);
        Thread shootThread = new Thread(shoot);

        //clawServo.setPosition(.48);
        intakeMotor2.setPower(0);

        //blinkin.setPattern(Lights.CustomPattern.RINGCOUNTER);


        telemetry.addData("Status", "Initialized");

        telemetry.update();

        //Wait for the game to start (driver presses PLAY)

        waitForStart();


        double speedReduce = 1;
        boolean toggle = false;
        boolean open = false;
        double pos = 1;
        double liftReduction = 55;
        double turretReduction = 1;
        double liftTarget = 0;
        int direction = 0;
        boolean notUsed = true;
        boolean toggle2 = false;
        int powerShot = 0;
        int reducedPower = 0;
        double shootHeading = 180;
        double heading = 0;
        double straight, strafe, rotate;
        int targetNum = 0;
        //lights.start();
        turretThread.start();
        armThread.start();
        //lights.start();
        shooter.start();
        drive.start();
        shootThread.start();

        while (opModeIsActive()) {
            //light section

//            if(counter.numBottomRings() != lastNumRings){
//                blinkin.begin();
//                blinkin.rings(counter.numBottomRings());
//            }

            //drive.update();
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //Open or close the claw
            if (this.gamepad2.a) {
                if (!toggle) {
                    open = !open;
                    toggle = true;
                } else {
                }
            } else {
                toggle = false;
            }

            if (open) {
                clawServo.setPosition(1);
            } else {
                clawServo.setPosition(.48);
            }


            // move the servo controlling the claw servo
            // either inside the robot or outside the robot
            if (gamepad2.left_stick_x < -.1) {
                liftRotateServo.setPosition(0);
            } else if (gamepad2.left_stick_x > .1) {
                liftRotateServo.setPosition(.68);
            }

            //lift code
            liftTarget = Range.clip(liftTarget, 0, 1110);

            if (Math.abs(gamepad2.left_stick_y) > .01) {
                liftTarget += -gamepad2.left_stick_y * liftReduction;
            }

            lift.setTargetPosition((int) liftTarget);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);

            //Drive code
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            gamepad1.left_stick_y * speedReduce,
//                            gamepad1.left_stick_x * speedReduce,
//                            -gamepad1.right_stick_x * speedReduce
//                    )
//            );

            //Pose2d currentPose = drive.getPoseEstimate();

//            if(gamepad1.a){
//
//                Trajectory shoot = drive.trajectoryBuilder(new Pose2d(currentPose.getX(), currentPose.getY(), Math.toRadians(currentPose.getHeading())), Math.toRadians(currentPose.getHeading()))
//                        .lineToLinearHeading(new Pose2d(-1,-2,Math.toRadians(180)))
//                        .build();
//
//                drive.followTrajectory(shoot);
//                //drive.turn(Math.toRadians(shootHeading));
//            }

//            if(gamepad1.x){
//                shootHeading += .05;
//            }else if(gamepad1.b){
//                shootHeading -= .05;
//            }


            if (gamepad1.dpad_down) {
                //intakeMotor1.setPower(0);
                intakeMotor2.setPower(0);
            } else if (gamepad1.left_bumper) {
                //intakeMotor1.setPower(1);
                intakeMotor2.setPower(1);
            } else if (gamepad1.left_trigger > 0) {
                //intakeMotor1.setPower(-1);
                intakeMotor2.setPower(-1);
            }

            //shooter code
            //This specific section of the shooter code is the code for the turret


//            if(gamepad1.y){
//                blinkin.cancel();
//            }else if(gamepad1.a){
//                blinkin.begin();
//            }

            if (gamepad1.right_bumper && shooterThread.isInThresh()) {
                shoot.activate();
            } else {
                shoot.deactivate();
            }

            if (gamepad2.start) {
                if (!toggle2) {
                    targetNum++;

                    targetNum %= 5;

                    if(targetNum < 0){
                        targetNum = 4;
                    }

                    switch (targetNum) {
                        case 0:
                            turret.setTarget(TurretCalculations.Target.RED_TOWER_GOAL);
                            break;
                        case 1:
                            turret.setTarget(TurretCalculations.Target.BLUE_TOWER_GOAL);
                            break;
                        case 2:
                            turret.setTarget(TurretCalculations.Target.RED_POWERSHOT1);
                            break;
                        case 3:
                            turret.setTarget(TurretCalculations.Target.RED_POWERSHOT2);
                            break;
                        case 4:
                            turret.setTarget(TurretCalculations.Target.RED_POWERSHOT3);
                            break;
                    }
                    toggle2 = true;
                }
            }else if(gamepad2.back){
                if (!toggle2) {
                    targetNum--;

                    targetNum %= 5;

                    if(targetNum < 0){
                        targetNum = 4;
                    }

                    switch (targetNum) {
                        case 0:
                            turret.setTarget(TurretCalculations.Target.RED_TOWER_GOAL);
                            break;
                        case 1:
                            turret.setTarget(TurretCalculations.Target.BLUE_TOWER_GOAL);
                            break;
                        case 2:
                            turret.setTarget(TurretCalculations.Target.RED_POWERSHOT1);
                            break;
                        case 3:
                            turret.setTarget(TurretCalculations.Target.RED_POWERSHOT2);
                            break;
                        case 4:
                            turret.setTarget(TurretCalculations.Target.RED_POWERSHOT3);
                            break;
                    }
                    toggle2 = true;
                }
            }else{
                toggle2 = false;
            }



            telemetry.addData("Status", Html.fromHtml(turret.getStatus(), 0));
//            telemetry.addData("Lift current position", lift.getTargetPosition());
//            telemetry.addData("Lift height", lift.getCurrentPosition());
//            telemetry.addData("Kicker pos", kicker.getPosition());
//            telemetry.addData("Turret target", pos);
            //telemetry.addData("Turret real", turret.pos);
//            telemetry.addData("Running?", running);
//            telemetry.addData("velocity", shooterThread.getVelocity());
            //telemetry.addData("is in range?", shooterThread.isInThresh());
            telemetry.addData("Target Velocity", shooterThread.velocity);
            telemetry.addData("corrected angle", turret.getAngleCorrection());
            telemetry.addData("tracking status", turret.isTracking());
            telemetry.addData("current target", turret.getCurrentTar());
            //telemetry.addData("bottom thresh", shooterThread.getBottomThresh());
            //telemetry.addData("top thresh", shooterThread.getTopThresh());
            //telemetry.addData("shoot status", shoot.getStatus());
//            telemetry.addData("heading", shootHeading);
//            telemetry.addData("bottomDist" , blinkin.bottomDist());
//            telemetry.addData("topDist", blinkin.topDist());
//            telemetry.addData("bottomNum" , counter.numBottomRings());
//            telemetry.addData("topNum", counter.numTopRings());
            //telemetry.addData("on?", blinkin.on);
            //telemetry.addData("lastNumRings", lastNumRings);
            telemetry.update();

            //lastNumRings = counter.numBottomRings();
        }
        turretThread.interrupt();
        arm.cancel();
        armThread.interrupt();
        shooter.interrupt();
        drive.interrupt();
        shootThread.interrupt();
        //lights.interrupt();
    }

}
