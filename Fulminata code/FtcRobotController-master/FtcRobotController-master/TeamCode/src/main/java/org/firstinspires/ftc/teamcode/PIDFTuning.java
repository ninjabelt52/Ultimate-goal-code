package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class PIDFTuning extends LinearOpMode {

    public void runOpMode(){
        int valueChange = 0;
        double pos = 1;
        double turretReduction = 0;
        boolean running = false;
        boolean toggle2 = false;
        boolean toggle1 = false;
        double p = .86,i = .126,d = 35,f = 12.6;

        DcMotorEx shooter;
        Servo turret, kicker;

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        turret = hardwareMap.get(Servo.class, "turret");
        kicker = hardwareMap.get(Servo.class, "kicker");

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            if(gamepad2.right_stick_x != 0){
                pos += gamepad2.right_stick_x * turretReduction;
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
                    //shooter.setPower(.92);
                    shooter.setVelocity(2440);
            }else{
                shooter.setPower(0);
            }

            if(gamepad2.right_trigger > 0 && running){
                kicker.setPosition(.55);
            }else{
                kicker.setPosition(1);
            }

            if(gamepad2.x){
                if(!toggle1){
                    valueChange ++;
                    toggle1 = true;
                }else{}
            }else{
                toggle1 = false;
            }

            if(valueChange == 0){
                telemetry.addData("Changing value", "p+ " + p);
                telemetry.addData("velocity", shooter.getVelocity());
                telemetry.addData("running?", running);
                telemetry.update();

                if(gamepad2.y) {
                    p += .001;
                }else if(gamepad2.a){
                    p-= .001;
                }
            }else if(valueChange == 1){
                telemetry.addData("Changing value", "i: "+ i);
                telemetry.addData("velocity", shooter.getVelocity());
                telemetry.addData("running?", running);
                telemetry.update();

                if(gamepad2.y) {
                    i += .001;
                }else if(gamepad2.a){
                    i-= .001;
                }
            }else if(valueChange == 2){
                telemetry.addData("Changing value", "d: " + d);
                telemetry.addData("velocity", shooter.getVelocity());
                telemetry.addData("running?", running);
                telemetry.update();

                if(gamepad2.y) {
                    d += .001;
                }else if(gamepad2.a){
                    d-= .001;
                }
            }else if(valueChange == 3){


                if(gamepad2.y) {
                    f += .001;
                }else if(gamepad2.a) {
                    f -= .001;
                }

                telemetry.addData("Changing value", "f: " + f);
                telemetry.addData("velocity", shooter.getVelocity());
                telemetry.addData("running?", running);
                telemetry.update();
            }else if(valueChange == 4){
                valueChange = 0;
            }

            if(gamepad2.right_stick_button)
            shooter.setVelocityPIDFCoefficients(p,i,d,f);
        }
    }
}
