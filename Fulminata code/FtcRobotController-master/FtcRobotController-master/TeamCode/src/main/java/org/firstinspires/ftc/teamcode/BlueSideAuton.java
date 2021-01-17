package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class BlueSideAuton extends LinearOpMode {
    OpenCvCamera webcam;
    RingDetermination.RingDeterminationPipeline rings;
    RingDetermination.RingDeterminationPipeline.RingPos analysis;
    DcMotor shooter;
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
    

    @Override
    public void runOpMode() {
        
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        turret = hardwareMap.get(Servo.class, "turret");
        kicker = hardwareMap.get(Servo.class, "kicker");
        rotate = hardwareMap.get(Servo.class, "rotate");
        claw = hardwareMap.get(Servo.class, "claw");
        liftinit = hardwareMap.get(DcMotor.class, "lift");
        m1 = hardwareMap.get(DcMotor.class, "Blw");
        m2 = hardwareMap.get(DcMotor.class, "Brw");
        m3 = hardwareMap.get(DcMotor.class, "Flw");
        m4 = hardwareMap.get(DcMotor.class, "Frw");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu.initialize(parameters);

        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

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
            
            //Shooter.startMotor(.97);

            telemetry.addData("Amount of rings", rings.getAnalysis());
            telemetry.addData("Average", rings.Avg());
            telemetry.update();


            lift.move(60);

            switch (analysis) {
                case NONE:
                    Drive.Backup(-.5, 0, 0, 1120);

                    Shooter.TurnTurret(250);
                    Shooter.shoot();
                    Shooter.TurnTurret(300);
                    Shooter.shoot();
                    Shooter.TurnTurret(350);
                    Shooter.shoot();

                    //Claw.open();

                    Drive.Drive(1, 0,0,1120);
                    Claw.fold();
                    Drive.TurnLeft(90);
                    Drive.Drive(-1, 0, 90, 560);

                    //Claw.open();
                    Claw.close();
                    lift.move(60);

                    Drive.Drive(1,0,90,560);
                    Drive.Drive(0,1,90,1120);

                    lift.move(0);
                    //Claw.open();
                case ONE:
                    Drive.Drive(-1, 0, 0, 1120);

                    Shooter.TurnTurret(250);
                    Shooter.shoot();
                    Shooter.TurnTurret(300);
                    Shooter.shoot();
                    Shooter.TurnTurret(350);
                    Shooter.shoot();

                    Drive.Drive(-1, 0, 0, 560);

                    Drive.TurnLeft(90);
                    lift.move(0);
                    //Claw.open();

                    Claw.fold();
                    Drive.Drive(0, -1, 0, 1680);
                    //Claw.open();
                    Drive.Drive(-1, 0, 0, 560);
                    Claw.close();
                    lift.move(60);
                    Claw.fold();

                    Drive.Drive(1, 0, 0, 560);
                    Drive.Drive(0, 1, 0, 1680);
                    //Claw.open();
                    lift.move(0);

                    Drive.Drive(0, -1, 0, 560);
                case FOUR:
                    Drive.Drive(0, -1, 0, 1120);

                    Shooter.TurnTurret(250);
                    Shooter.shoot(3);

                    Drive.Drive(-1, 0, 0, 1120);
                    //Claw.open();
                    Claw.fold();

                    Drive.Drive(1, 0, 0, 2240);
                    Drive.TurnLeft(90);
                    lift.move(0);
                    //Claw.open();
                    Drive.Drive(-1, 0, 90, 560);
                    Claw.close();
                    lift.move(60);
                    Claw.fold();

                    Drive.Drive(1, 0, 90,560);
                    Drive.Drive(0, 1, 90, 2240);
                    Drive.TurnRight(0);

                    //Claw.open();
                    Claw.fold();

                    Drive.Drive(1, 0, 0, 1120);
            }
        }

    }