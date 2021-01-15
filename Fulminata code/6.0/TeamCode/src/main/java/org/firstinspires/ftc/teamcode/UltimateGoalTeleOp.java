package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "TeleOpMode")
public class UltimateGoalTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor Blw, Brw, Flw, Frw, lift, intakeMotor;
        Servo liftRotateServo;
        Servo clawServo;
        DcMotor turret;
        DcMotor shooter;
        Servo kicker;

        Blw = hardwareMap.get(DcMotor.class, "Blw");
        Brw = hardwareMap.get(DcMotor.class, "Brw");
        Flw = hardwareMap.get(DcMotor.class, "Flw");
        Frw = hardwareMap.get(DcMotor.class, "Frw");
        lift = hardwareMap.get(DcMotor.class, "lift");
        turret = hardwareMap.get(DcMotor.class, "turret");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        liftRotateServo = hardwareMap.get(Servo.class, "rotate");
        clawServo = hardwareMap.get(Servo.class, "claw");

        Blw.setDirection(DcMotor.Direction.REVERSE);
        Flw.setDirection(DcMotor.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        //Wait for the game to start (driver presses PLAY)

        waitForStart();

        double speed;
        double rotation;
        double strafe;
        double speedReduce = 1;
        boolean toggle = true;
        boolean open = false;
        boolean toggle2 = true;
        boolean running = true;
        double pos = 0;
        double liftReduction = 1;
        double turretReduction = 1;

        while (opModeIsActive()) {
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                clawServo.setPosition(0);
            }



            // move the servo controlling the claw servo either inside the robot or outside the robot
            if(gamepad2.dpad_left){
                liftRotateServo.setPosition(0);
            } else if(gamepad2.dpad_right){
                liftRotateServo.setPosition(.5);
            }


            //reduce the speed if so desired
            if(gamepad1.right_trigger > 0){
                speedReduce = .5;
            } else {
                speedReduce = 1;
            }



            //lift code
            if(-gamepad2.right_stick_y > 0){
                if(lift.getCurrentPosition() < 1100){
                    lift.setPower(-gamepad2.right_stick_y * liftReduction);
                }else{
                    lift.setPower(0);
                }
            }else if(-gamepad2.right_stick_y < 0){
                if(lift.getCurrentPosition() > 40){
                    lift.setPower(-gamepad2.right_stick_y * liftReduction);
                }else{
                    lift.setPower(0);
                }
            }else{
                lift.setPower(0);
            }

            if(lift.getCurrentPosition() > 900 || lift.getCurrentPosition() < 150){
                liftReduction = .5;
            }else{
                liftReduction = 1;
            }


            //Drive code
            speed = -gamepad1.left_stick_y;
            rotation = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            Blw.setPower((speed - strafe + rotation) * speedReduce);
            Brw.setPower((speed + strafe - rotation) * speedReduce);
            Flw.setPower((speed + strafe + rotation) * speedReduce);
            Frw.setPower((speed - strafe - rotation) * speedReduce);

            //intake motor
            if(gamepad1.left_bumper || gamepad2.left_bumper){
                if(!toggle2){
                    running = !running;
                    toggle2 = true;
                }
            }else{
                toggle2 = false;
            }

            if(running) {
                intakeMotor.setPower(1);
            }

            //shooter code
            //This specific section of the shooter code is the code for the turret

            if(gamepad2.right_stick_x != 0){
                pos += gamepad2.right_stick_x;
            }

            /*if(gamepad2.left_trigger > 0){
                turretReduction = .25;
            }else{
                turretReduction = .5;
            }

            if(gamepad2.right_stick_x < 0){
                if(turret.getCurrentPosition() < 250){
                    turret.setPower(gamepad2.right_stick_x * turretReduction);
                }else{
                    turret.setPower(0);
                }
            }else if(gamepad2.right_stick_x > 0){
                if(turret.getCurrentPosition() > 0){
                    turret.setPower(gamepad2.right_stick_x * turretReduction);
                }else{
                    turret.setPower(0);
                }
            }else{
                turret.setPower(0);
            }*/

           /* if(gamepad2.right_stick_x > 0){
                pos += .01;
            }

            if(gamepad2.right_stick_x < 0){
                pos -= .01;
            }*/

            Range.clip(pos,0,1);

            turret.setTargetPosition((int)pos);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //This section of the shooter code revs up the shooter motor and activates the kicker servo at the touch of a button

            //shooter.setPower(1);

            if(gamepad2.right_bumper){
                kicker.setPosition(.65);
            }else{
                kicker.setPosition(1);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Lift current position", lift.getTargetPosition());
            telemetry.addData("Lift height", lift.getCurrentPosition());
            telemetry.addData("Kicker pos", kicker.getPosition());
            telemetry.update();
        }
    }

}
