package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class UltimateGoalAutonomous extends LinearOpMode {
    OpenCvCamera webcam;
    RingDetermination.RingDeterminationPipeline rings;
    RingDetermination.RingDeterminationPipeline.RingPos analysis;
    DcMotorEx shooter;
    Servo turret;
    Servo kicker;
    Servo rotate;
    Servo claw;
    DcMotor liftinit;
    DcMotor m1;
    DcMotor m2;
    DcMotor m3;
    DcMotor m4;
    BNO055IMU imu;
    DistanceSensor distanceSensor;
    DcMotor intake;
    

    @Override
    public void runOpMode() throws InterruptedException{
        
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        turret = hardwareMap.get(Servo.class, "turret");
        kicker = hardwareMap.get(Servo.class, "kicker");
        rotate = hardwareMap.get(Servo.class, "rotate");
        claw = hardwareMap.get(Servo.class, "claw");
        liftinit = hardwareMap.get(DcMotor.class, "lift");
        m1 = hardwareMap.get(DcMotor.class, "Blw");
        m2 = hardwareMap.get(DcMotor.class, "Flw");
        m3 = hardwareMap.get(DcMotor.class, "Brw");
        m4 = hardwareMap.get(DcMotor.class, "Frw");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        intake = hardwareMap.get(DcMotor.class, "intake");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu.initialize(parameters);

        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setVelocityPIDFCoefficients(4.724,0.136,.432,12.6);

        wobbleGoal Claw = new wobbleGoal(claw, rotate);
        MecanumDrivetrain Drive = new MecanumDrivetrain(m1, m2, m3, m4, imu);
        Lift lift = new Lift(liftinit);
        Shooter Shooter = new Shooter(shooter, turret, kicker);



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
            telemetry.addLine("Initialized");
            telemetry.addData("Amount of rings", rings.getAnalysis());
            telemetry.addData("Average", rings.Avg());
            telemetry.update();
        }

        waitForStart();
            analysis = rings.getAnalysis();
            

            telemetry.addData("Amount of rings", rings.getAnalysis());
            telemetry.addData("Average", rings.Avg());
            telemetry.update();


            lift.move(580);
            //turn turret to start motor
            while(shooter.getVelocity() < 1000){
                Shooter.startMotor(2275);
            }


            switch (analysis) {
                case NONE:
                    Shooter.startMotor(2275);
                    Drive.Backup(0,-.25,0,-1120);
                    sleep(500);
                    //Drive.Drive(.5,0,-20,740);

                    Shooter.TurnTurret(.546);
                    //Shooter.TurnTurret(.185);
//                    sleep(250);
//                    Shooter.shoot();
//                    sleep(250);
//                    Shooter.retract();

                    sleep(500);
                    for(int i = 0; i < 3; i++){
                        Shooter.shoot();
                        sleep(500);
                        Shooter.retract();
                        sleep(500);
                    }

                    //sleep(500);
                    Shooter.stopMotor();
                    //sleep(500);
                    //Drive to deliver wobble #1
                    Drive.Backup(0,-.25,0,-420);
                    sleep(500);
                    Drive.TurnRight(170);
                    sleep(500);
                    Drive.Backup(-.25,0,-170,-150);

                    Claw.openArm();
                    sleep(500);
                    lift.move(0);
                    sleep(500);
                    Claw.openClaw();
                    sleep(500);

                    Drive.Backup(0,-.25,-170,-360);
                    sleep(500);
                    Drive.TurnLeft(0);
                    sleep(500);
                    Drive.Drive(0,.25,0,480);
                    //Drive into wall
                    Drive.Drive(.5,0,-20,490);
                    //open arm to grab wobble
                    Claw.openArm();
                    Claw.openClaw();
                    sleep(500);
                    //drive off wall
                    Drive.Backup(-.5,0,0,-5);
                    sleep(500);
                    //drive to wobble
                    Drive.Drive(0,.25,0,400);
                    sleep(500);
                    Claw.close();
                    sleep(1000);
                    lift.move(580);
                    Claw.fold();

                    //backup to target zone
                    Drive.Backup(0,-.25,0,-1055);
                    sleep(500);
                    //strafe to position the claw in the right direction
                    Drive.Backup(-.25,0,0,-500);
                    Drive.TurnRight(170);
                    //get in position to drop wobble goal
                    Drive.Drive(0,.25,-170,280);
                    Claw.openArm();
                    sleep(500);
                    lift.move(0);
                    Claw.openClaw();
                    //wait for wobble #2 to stop wobbling
                    sleep(500);
                    //Drive off of wobble goal
                    Drive.Drive(.25,0,-170,280);

                    sleep(500);

                    break;

                    //Drive.Backup(0,-.25,180,-250);
//                    Drive.TurnLeft(90);
//                    Drive.Drive(-1, 0, 90, 560);
//
//                    Claw.open();
//                    Claw.close();
//                    lift.move(60);
//
//                    Drive.Drive(1,0,90,560);
//                    Drive.Drive(0,1,90,1120);
//
//                    lift.move(0);
//                    Claw.open();
                case ONE:
                    //drive out of way of ring
                    Drive.Backup(-.25, 0,5,-320);
                    sleep(250);
                    //backup to shoot
                    Drive.Backup(0,-.75,0,-1100);
                    sleep(500);
                    //drive to shoot
                    Drive.Drive(.25,0,-25,450);
                    sleep(250);

                    Shooter.TurnTurret(.55);
                    sleep(500);
                    for(int i = 0; i < 3; i++){
                        Shooter.shoot();
                        sleep(500);
                        Shooter.retract();
                        sleep(500);
                    }
                    intake.setPower(1);
                    Drive.Drive(0,.25,0,560);
                    sleep(250);
                    Drive.Backup(0,-.25,0,-460);

                    Shooter.TurnTurret(.55);
                    sleep(1500);

                    //shoot last ring
                    Shooter.shoot();
                    sleep(500);
                    Shooter.retract();
                    sleep(500);


                    //stop everything
                    sleep(500);
                    Shooter.stopMotor();
                    intake.setPower(0);

                    //backup into target zone
                    Drive.Backup(0,-.25,0,-920);
                    Drive.Drive(.5,0,-35,150);

                    //drop wobble goal
                    sleep(500);
                    Claw.openArm();
                    sleep(500);
                    lift.move(0);
                    Claw.openClaw();
                    sleep(500);

                    //move off wobble goal #1
                    Drive.Drive(.25,0,-20,180);
                    Drive.Drive(0,.75,0,1320);
                    sleep(500);
                    //drive into wall
                    Drive.Drive(.25,0,-20,105);
                    //drive off wall
                    Drive.Backup(-.25,0,0,-5);

                    //drive slowly to wobble #2
                    Drive.Drive(0,.25,0,380);
                    sleep(500);
                    Claw.close();
                    sleep(500);

                    // move wobble up
                    lift.move(580);
                    //strafe off wall
                    Drive.Backup(-.5,0,0,-130);
                    //backup to target zone
                    Drive.Backup(0,-.75,0,-1720);
                    lift.move(0);
                    //drop wobble
                    Claw.openClaw();
                    sleep(500);

                    //drive off wobble #2
                    Drive.Drive(.5,0,-20,100);
                    Claw.fold();
                    //HOME FREE!!!
                    Drive.Drive(0,.5,0,380);

                    break;
                case FOUR:
                    //slow down motor
                    Shooter.startMotor(2350);

                    //drive out of way of ring
                    Drive.Backup(-.35, 0,10,-320);
                    sleep(250);
                    //backup to shoot
                    Drive.Backup(0,-.75,0,-1000);
                    sleep(500);
                    //drive to shoot
                    Drive.Drive(.35,0,-22,550);

//                    telemetry.addData("gyro", Drive.gyro());
//                    telemetry.update();
//                    sleep(1000);
//                    if(Drive.gyro() > 0){
//                        Drive.TurnRight(0);
//                    }else if(Drive.gyro() < 0){
//                        Drive.TurnLeft(0);
//                    }

                    Shooter.TurnTurret(.67);
                    sleep(500);
                    for(int i = 0; i < 3; i++){
                        Shooter.shoot();
                        sleep(300);
                        Shooter.retract();
                        sleep(300);
                    }
                    intake.setPower(1);
                    //pick up rings
                    Drive.Drive(0,.15,0,500);
                    sleep(250);
                    Drive.Backup(0,-.25,0,-460);

                    Shooter.TurnTurret(.64);
                    sleep(500);

                    //shoot last 3 rings

                    for(int i = 0; i < 3; i++) {
                        Shooter.shoot();
                        sleep(300);
                        Shooter.retract();
                        sleep(300);
                    }


                    //stop the shooter
                    sleep(500);
                    Shooter.stopMotor();
                    intake.setPower(0);
                    //backup into target zone
                    Drive.Backup(0,-.5,0,-1020);

                    //deliver wobble #1
                    sleep(250);
                    Drive.TurnRight(180);
                    sleep(500);
                    Drive.Backup(-.3,0,-180,-200);
                    Claw.openArm();
                    sleep(250);
                    lift.move(0);
                    Claw.openClaw();
                    sleep(250);

                    //drive off wobble #1
                    Drive.Drive(.5,0,-200,760);
                    sleep(250);

                    //backup to wobble #2
                    Drive.Backup(0,-.75,-180,-2160);
                    sleep(500);
                    //strafe to wobble goal
                    Drive.Backup(-.3,0,-180,-470);
                    sleep(250);
                    //grab wobble #2
                    intake.setPower(1);
                    Claw.close();
                    sleep(500);
                    lift.move(580);
                    //sleep(500);

                    //drive to zone c
                    Drive.Drive(0,.75,-180,2040);

                    Drive.Backup(-.5,0,-180,-580);
                    sleep(500);
                    lift.move(0);
                    sleep(500);
                    Claw.openClaw();
                    sleep(1);
                    lift.move(580);

                    //park
                    Drive.Backup(0,-.5,-180,-860);
                    lift.move(0);
                    break;
            }

            Drive.Stop();
            sleep(500);
        }


    }