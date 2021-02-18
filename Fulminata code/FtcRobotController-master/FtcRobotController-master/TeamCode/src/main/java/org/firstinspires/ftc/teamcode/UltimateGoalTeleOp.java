package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "TeleOpMode")
public class UltimateGoalTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor Blw, Brw, Flw, Frw, lift, intakeMotor;
        Servo liftRotateServo;
        Servo clawServo;
        Servo turret;
        DcMotorEx shooter;
        Servo kicker;

        Blw = hardwareMap.get(DcMotor.class, "Blw");
        Brw = hardwareMap.get(DcMotor.class, "Brw");
        Flw = hardwareMap.get(DcMotor.class, "Flw");
        Frw = hardwareMap.get(DcMotor.class, "Frw");
        lift = hardwareMap.get(DcMotor.class, "lift");
        turret = hardwareMap.get(Servo.class, "turret");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        liftRotateServo = hardwareMap.get(Servo.class, "rotate");
        clawServo = hardwareMap.get(Servo.class, "claw");

        Blw.setDirection(DcMotor.Direction.REVERSE);
        Flw.setDirection(DcMotor.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //this is the ideal pidf values for a gobilda 5202 1:1 motor
        shooter.setVelocityPIDFCoefficients(4.724,.136,.432,12.6);

        clawServo.setPosition(.48);
        liftRotateServo.setPosition(.68);
        shooter.setPower(0);
        intakeMotor.setPower(0);


        telemetry.addData("Status", "Initialized");

        telemetry.update();

        //Wait for the game to start (driver presses PLAY)

        waitForStart();

        double speed;
        double rotation;
        double strafe;
        double speedReduce = 1;
        boolean toggle = false;
        boolean open = false;
        boolean toggle2 = false;
        boolean running = false;
        double pos = 1;
        double liftReduction = 16;
        double turretReduction = 1;
        double liftTarget = 0;
        int direction = 0;
        boolean toggle3 = false;
        int powerShot = 0;
        int reducedPower = 0;

        while (opModeIsActive()) {
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



            // move the servo controlling the claw servo either inside the robot or outside the robot
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
            speed = gamepad1.left_stick_y;
            rotation = gamepad1.right_stick_x;
            strafe = -gamepad1.left_stick_x;

            Blw.setPower((speed - strafe + rotation) * speedReduce);
            Brw.setPower((speed + strafe - rotation) * speedReduce);
            Flw.setPower((speed + strafe + rotation) * speedReduce);
            Frw.setPower((speed - strafe - rotation) * speedReduce);


            if(gamepad1.dpad_down){
                direction = 0;
            }else if(gamepad1.left_bumper){
                direction = 1;
            }else if(gamepad1.left_trigger > 0){
                direction = 2;
            }

            if(direction == 0){
                intakeMotor.setPower(0);
            }else if(direction == 1){
                intakeMotor.setPower(1);
            }else if(direction == 2){
                intakeMotor.setPower(-1);
            }

            //shooter code
            //This specific section of the shooter code is the code for the turret

            if(gamepad2.right_stick_x != 0){
                pos += gamepad2.right_stick_x * turretReduction;
            }else if(gamepad2.x){
                pos = .674;
            }

            if(gamepad2.right_bumper){
                turretReduction = .005;
            }else{
                turretReduction = .02;
            }

            pos = Range.clip(pos,0, 1);

            turret.setPosition(pos);

            //This section of the shooter code revs up the shooter motor and activates the kicker servo at the touch of a button

            if(gamepad2.b){
                if(!toggle2){
                    running = !running;
                    toggle2 = true;
                }else{}
            }else{
                toggle2 = false;
            }

            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(running) {
                if (gamepad2.left_trigger > 0) {
                    shooter.setVelocity(reducedPower);
                } else {
                    //shooter.setPower(.92);
                    shooter.setVelocity(2500);
                }
            }else{
                shooter.setPower(0);
            }

            if(gamepad2.y){
                if(!toggle3){
                    powerShot++;
                    toggle3 = true;
                }
            }else{
                toggle3 = false;
            }

            if(powerShot == 1 && gamepad2.y){
                pos = .573;
                reducedPower = 2250;
                telemetry.speak("left");
                telemetry.update();
            }else if(powerShot == 2 && gamepad2.y){
                pos = .595;
                reducedPower = 2350;
                telemetry.speak("center");
                telemetry.update();
            }else if(powerShot == 3 && gamepad2.y){
                pos = .618;
                reducedPower = 2280;
                telemetry.speak("right");
                telemetry.update();
            }else if (powerShot == 4){
                powerShot = 1;
            }

            if(gamepad2.right_trigger > 0 && running && shooter.getVelocity() >= 2400){
                kicker.setPosition(.55);
            }else{
                kicker.setPosition(1);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Lift current position", lift.getTargetPosition());
            telemetry.addData("Lift height", lift.getCurrentPosition());
            telemetry.addData("Kicker pos", kicker.getPosition());
            telemetry.addData("Turret target", pos);
            telemetry.addData("Turret real", turret.getPosition());
            telemetry.addData("Running?", running);
            telemetry.addData("velocity", shooter.getVelocity());
            telemetry.update();
        }
    }

}
