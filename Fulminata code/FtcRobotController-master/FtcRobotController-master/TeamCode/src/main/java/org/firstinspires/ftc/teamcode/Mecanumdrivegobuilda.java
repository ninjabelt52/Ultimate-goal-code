package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;


/**

 * Created by Boen on 10-10-19

 */

@TeleOp
@Disabled


public class Mecanumdrivegobuilda extends LinearOpMode {

    private DcMotor Blw;
    private DcMotor Brw;
    private DcMotor Flw;
    private DcMotor Frw;
    @Override

    public void runOpMode() throws InterruptedException {
        Blw = hardwareMap.get(DcMotor.class, "Blw");
        Brw = hardwareMap.get(DcMotor.class, "Brw");
        Flw = hardwareMap.get(DcMotor.class, "Flw");
        Frw = hardwareMap.get(DcMotor.class, "Frw");

        Blw.setDirection(DcMotor.Direction.REVERSE);
        //Brw.setDirection(DcMotor.Direction.REVERSE);
        Flw.setDirection(DcMotor.Direction.REVERSE);
        //Frw.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");

        telemetry.update();

        //Wait for the game to start (driver presses PLAY)

        waitForStart();


        //run until the end of the match (driver presses STOP)

        double speed;
        double rotation;
        double strafe;
        while (opModeIsActive()) {


            speed = -gamepad1.left_stick_y;

            rotation = gamepad1.right_stick_x;

            strafe = gamepad1.left_stick_x;


            Blw.setPower(speed - strafe + rotation);

            Brw.setPower(speed + strafe - rotation);

            Flw.setPower(speed + strafe + rotation);

            Frw.setPower(speed - strafe - rotation);

            telemetry.addData("Status", "Running");
            telemetry.update();
            }
        }

    }


