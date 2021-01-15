package org.firstinspires.ftc.teamcode;





import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by Boen on 10-10-19
 */

@TeleOp(name = "Demo", group = "competition modes")

public class demo_mode extends LinearOpMode {
    private Gyroscope imu;
    private DcMotorEx backLeftWheel, backRightWheel, frontLeftWheel, frontRightWheel, linearLift, linearLift2;
    private Servo CLAW, trayServoL, trayServoR, Capstone;
    double startPosition;
    int goalLiftHeight = 0;
    boolean upButton;
    boolean downButton;
    double liftMultiplier = 1;
    boolean open = false;
    double totalTicks;

    @Override


    public void runOpMode() throws InterruptedException {


        imu = hardwareMap.get(Gyroscope.class, "imu");

        backLeftWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Back_left_wheel");

        backRightWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Back_right_wheel");

        frontLeftWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Front_left_wheel");

        frontRightWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "Front_right_wheel");

        linearLift = (DcMotorEx) hardwareMap.get(DcMotor.class, "linearLift");

        linearLift2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "linearLift2");

        CLAW = hardwareMap.servo.get("CLAW");

        trayServoL = hardwareMap.servo.get("trayServoL");

        trayServoR = hardwareMap.servo.get("trayServoR");

        Capstone = hardwareMap.get(Servo.class, "Capstone");


        //backLeftWheel.setDirection(DcMotor.Direction.REVERSE);


        //backRightWheel.setDirection(DcMotor.Direction.REVERSE);


        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);


        //frontRightWheel.setDirection(DcMotor.Direction.REVERSE);


        linearLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        linearLift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        linearLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        linearLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        CLAW.setPosition(0);
        Capstone.setPosition(1);
        trayServoL.setPosition(0);
        trayServoR.setPosition(1);


        telemetry.addData("Status", "Initialized");


        telemetry.update();


        //Wait for the game to start (driver presses PLAY)


        waitForStart();


        //run until the end of the match (driver presses STOP)


        double speed = 0;

        double rotation;

        double strafe;

        double multiplier = 0;


        while (opModeIsActive()) {


            // Drive code


            speed = this.gamepad1.left_stick_y * multiplier;


            rotation = -this.gamepad1.left_stick_x * multiplier;


            strafe = -this.gamepad1.right_stick_x * multiplier;
            backLeftWheel.setPower(speed + strafe - rotation);
            backRightWheel.setPower(-speed + strafe - rotation);
            frontLeftWheel.setPower(speed + strafe + rotation);
            frontRightWheel.setPower(-speed + strafe + rotation);
            if (gamepad1.left_trigger > .5) {

                multiplier = .2;

            } else {

                multiplier = .5;

            }


            // CLAAAAW code

            if (this.gamepad2.a) {


                CLAW.setPosition(1);


            } else if (this.gamepad2.y) {


                CLAW.setPosition(0);


            }


            if (this.gamepad1.left_bumper) {

                trayServoL.setPosition(1);

                trayServoR.setPosition(0);

            } else {

                trayServoL.setPosition(0);

                trayServoR.setPosition(1);

            }


            //**

            //This code is for slowing down the lift, if we don't use it, then just un-comment out this code ^^ above


            if (gamepad2.right_trigger > .25) {

                liftMultiplier = .5;

            } else {

                liftMultiplier = 1;

            }

            //*/


            int currentPosition = linearLift.getCurrentPosition();

            double stickHeight = this.gamepad2.left_stick_y;


            if (currentPosition <= -1600) {

                if (stickHeight > .25) {

                    linearLift.setPower(stickHeight * liftMultiplier);

                    linearLift2.setPower(stickHeight * liftMultiplier);

                } else {

                    linearLift.setPower(0);

                    linearLift2.setPower(0);

                }

            } else if (currentPosition >= -100) {

                if (stickHeight < -.25) {

                    linearLift.setPower(stickHeight * liftMultiplier);

                    linearLift2.setPower(stickHeight * liftMultiplier);

                } else {

                    linearLift.setPower(0);

                    linearLift2.setPower(0);

                }

            } else {

                if ((stickHeight > .25) && !(currentPosition >= 0)) {

                    linearLift.setPower(stickHeight * liftMultiplier);

                    linearLift2.setPower(stickHeight * liftMultiplier);

                } else if ((stickHeight < -0.25) && !(currentPosition <= -1600)) {

                    linearLift.setPower(stickHeight * liftMultiplier);

                    linearLift2.setPower(stickHeight * liftMultiplier);

                } else {

                    linearLift.setPower(0);

                    linearLift2.setPower(0);

                }

            }

                linearLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                linearLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





            telemetry.addData("CLAAAAAW position", CLAW.getPosition());

            telemetry.addData("Target Power", speed);

            telemetry.addData("Motor Power", backLeftWheel.getPower());

            telemetry.addData("Status", "Running");

            telemetry.addData("Linear lift height", linearLift.getCurrentPosition());


            telemetry.update();


        }


    }

}



