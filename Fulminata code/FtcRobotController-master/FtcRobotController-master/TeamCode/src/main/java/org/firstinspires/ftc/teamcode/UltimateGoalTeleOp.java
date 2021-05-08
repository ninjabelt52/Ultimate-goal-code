package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp (name = "TeleOpMode")
public class UltimateGoalTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry telemetry = dash.getTelemetry();
        DcMotor Blw, Brw, Flw, Frw, lift, intakeMotor1,intakeMotor2;
        Servo liftRotateServo;
        Servo clawServo;
        Servo turret;
        //RingCounter counter;
        //DcMotorEx shooter;
        Servo kicker;
        //PIDFController pid = new PIDFController(1,1,.06,1);

//        Blw = hardwareMap.get(DcMotor.class, "Blw");
//        Brw = hardwareMap.get(DcMotor.class, "Brw");
//        Flw = hardwareMap.get(DcMotor.class, "Flw");
//        Frw = hardwareMap.get(DcMotor.class, "Frw");
        lift = hardwareMap.get(DcMotor.class, "lift");
        turret = hardwareMap.get(Servo.class, "turret");
        //shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        //intakeMotor1 = hardwareMap.get(DcMotor.class, "intake1");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake1");
        liftRotateServo = hardwareMap.get(Servo.class, "rotate");
        clawServo = hardwareMap.get(Servo.class, "claw");
        //counter = new RingCounter(hardwareMap);
        //Lights blinkin = new Lights(hardwareMap, gamepad1, kicker);
        //Thread lights = new Thread(blinkin);

//        Blw.setDirection(DcMotor.Direction.REVERSE);
//        Flw.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //this is the ideal pidf values for a gobilda 5202 1:1 motor
        //shooter.setVelocityPIDFCoefficients(4.724,.136,.432,12.6);

        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
        ShooterThread shooterThread = new ShooterThread(hardwareMap);
        KnockDownArm arm = new KnockDownArm(hardwareMap, gamepad1);
        Thread armThread = new Thread(arm);
        Thread shooter = new Thread(shooterThread);

        //clawServo.setPosition(.48);
        //liftRotateServo.setPosition(.68);
        //shooter.setPower(0);
        //intakeMotor1.setPower(0);
        intakeMotor2.setPower(0);

        //blinkin.setPattern(Lights.CustomPattern.RINGCOUNTER);


        telemetry.addData("Status", "Initialized");

        telemetry.update();

        //Wait for the game to start (driver presses PLAY)

        waitForStart();


        double speedReduce = 1;
        boolean toggle = false;
        boolean open = false;
        boolean toggle2 = false;
        boolean running = false;
        double pos = 1;
        double liftReduction = 55;
        double turretReduction = 1;
        double liftTarget = 0;
        int direction = 0;
        int velocity = 1250;
        boolean notUsed = true;
        boolean toggle3 = false;
        int powerShot = 0;
        int reducedPower = 0;
        double shootHeading = 180;
        double heading = 0;
        //int lastNumRings = 0;

//        pid.setP(1);
//        pid.setI(1);
//        pid.setD(.06);
//        pid.setF(1);
        //lights.start();
        armThread.start();
        //lights.start();
        shooter.start();

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

            if(open){
                clawServo.setPosition(1);
            } else{
                clawServo.setPosition(.48);
            }



            // move the servo controlling the claw servo
            // either inside the robot or outside the robot
            if(gamepad2.dpad_left){
                liftRotateServo.setPosition(0);
            } else if(gamepad2.dpad_right){
                liftRotateServo.setPosition(.68);
            }


            //reduce the speed if so desired
            if(gamepad1.right_trigger > 0){
                speedReduce = .5;
            } else {
                speedReduce = .9;
            }



            //lift code

            liftTarget = Range.clip(liftTarget, 0, 1110);

            if(Math.abs(gamepad2.left_stick_y) > .01){
                liftTarget += -gamepad2.left_stick_y * liftReduction;
            }

            lift.setTargetPosition((int)liftTarget);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);


            //Drive code
            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y * speedReduce,
                            gamepad1.left_stick_x * speedReduce,
                            -gamepad1.right_stick_x * speedReduce
                    )
            );

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


            if(gamepad1.dpad_down){
                direction = 0;
            }else if(gamepad1.left_bumper){
                direction = 1;
            }else if(gamepad1.left_trigger > 0){
                direction = 2;
            }

            if(direction == 0){
                //intakeMotor1.setPower(0);
                intakeMotor2.setPower(0);
            }else if(direction == 1){
                //intakeMotor1.setPower(1);
                intakeMotor2.setPower(1);
            }else if(direction == 2){
                //intakeMotor1.setPower(-1);
                intakeMotor2.setPower(-1);
            }

            //shooter code
            //This specific section of the shooter code is the code for the turret

            if(gamepad2.right_stick_x != 0){
                pos += gamepad2.right_stick_x * turretReduction;
            }else if(gamepad2.x){
                pos =.99;
            }

            if(gamepad2.right_bumper){
                turretReduction = .005;
            }else{
                turretReduction = .04;
            }

            pos = Range.clip(pos,0, 1);

            turret.setPosition(pos);

            //This section of the shooter code revs up the shooter
            // motor and activates the kicker servo at the touch of a button

            if(gamepad1.b){
                if(!toggle2){
                    running = !running;
                    toggle2 = true;
                }else{}
            }else{
                toggle2 = false;
            }

//            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(running) {
                if (gamepad2.left_bumper) {
                    //pid.setSetPoint(reducedPower);
                    //shooter.setVelocity(pid.calculate(shooter.getVelocity(), reducedPower));

                    shooterThread.startMotor(1100);

                    //shooter.setVelocity(reducedPower);
                } else if(gamepad2.y) {
                    shooterThread.startMotor(1000);
                }else{
                        //shooter.setPower(.92);
//                    pid.setSetPoint(2500);
//                    shooter.setVelocity(pid.calculate(shooter.getVelocity(), 2500));
                        //shooter.setVelocity(2500);

                        shooterThread.startMotor(velocity);
                    }
            }else{
                //shooter.setPower(0);

                shooterThread.stopMotor();
            }

            if(gamepad2.dpad_up){
                if(!notUsed){
                    velocity = velocity - velocity % 50;
                    velocity += 50;
                    notUsed = true;
                }else{}
            }else if(gamepad2.dpad_down){
                if(!notUsed){
                    velocity = velocity - velocity % 50;
                    velocity -= 50;
                    notUsed = true;
                }
            }else if(gamepad2.dpad_down && gamepad2.left_trigger > 0){
                if(!notUsed){
                    velocity -= 10;
                    notUsed = true;
                }
            }else if(gamepad2.dpad_up && gamepad2.left_trigger > 0){
                if(!notUsed){
                    velocity += 10;
                    notUsed = true;
                }
            }else{
                notUsed = false;
            }

//            if(gamepad2.y){
//                if(!toggle3){
//                    powerShot++;
//                    toggle3 = true;
//                }
//            }else{
//                toggle3 = false;
//            }
//
//            if(powerShot == 1 && gamepad2.y){
//                pos = .573;
//                reducedPower = 2250;
//                telemetry.speak("left");
//                telemetry.update();
//            }else if(powerShot == 2 && gamepad2.y){
//                pos = .595;
//                reducedPower = 2350;
//                telemetry.speak("center");
//                telemetry.update();
//            }else if(powerShot == 3 && gamepad2.y){
//                pos = .618;
//                reducedPower = 2280;
//                telemetry.speak("right");
//                telemetry.update();
//            }else if (powerShot == 4){
//                powerShot = 1;
//            }

//            if(gamepad1.y){
//                blinkin.cancel();
//            }else if(gamepad1.a){
//                blinkin.begin();
//            }

            if(gamepad1.right_bumper && shooterThread.isInThresh()){
                kicker.setPosition(.55);
            }else{
                kicker.setPosition(1);
            }



            telemetry.addData("Status", "Running");
//            telemetry.addData("Lift current position", lift.getTargetPosition());
//            telemetry.addData("Lift height", lift.getCurrentPosition());
//            telemetry.addData("Kicker pos", kicker.getPosition());
            telemetry.addData("Turret target", pos);
            telemetry.addData("Turret real", turret.getPosition());
            telemetry.addData("Running?", running);
            telemetry.addData("velocity", shooterThread.getVelocity());
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
        arm.cancel();
        armThread.interrupt();
        shooter.interrupt();
        //lights.interrupt();
    }

}
