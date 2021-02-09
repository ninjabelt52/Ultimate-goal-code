package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
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
        double p = 4.724,i = 0.136,d = .432,f = 12.6;
        double changeValue;
        int velocity = 2560, lastvelocity = velocity;
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
                    shooter.setVelocity(lastvelocity);

                    if(gamepad2.right_stick_button){
                        lastvelocity = velocity;
                    }
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

            if(gamepad2.right_trigger > 0){
                changeValue = .1;
            }else{
                changeValue = .001;
            }

            if(valueChange == 0){
                telemetry.addData("Changing value", "p+ " + p);
                telemetry.addData("velocity", shooter.getVelocity());
                telemetry.addData("running?", running);
                telemetry.update();

                if(gamepad2.y) {
                    p += changeValue;
                }else if(gamepad2.a){
                    p-= changeValue;
                }
            }else if(valueChange == 1){
                telemetry.addData("Changing value", "i: "+ i);
                telemetry.addData("velocity", shooter.getVelocity());
                telemetry.addData("running?", running);
                telemetry.update();

                if(gamepad2.y) {
                    i += changeValue;
                }else if(gamepad2.a){
                    i-= changeValue;
                }
            }else if(valueChange == 2){
                telemetry.addData("Changing value", "d: " + d);
                telemetry.addData("velocity", shooter.getVelocity());
                telemetry.addData("running?", running);
                telemetry.update();

                if(gamepad2.y) {
                    d += changeValue;
                }else if(gamepad2.a){
                    d-= changeValue;
                }
            }else if(valueChange == 3){


                if(gamepad2.y) {
                    f += changeValue;
                }else if(gamepad2.a) {
                    f -= changeValue;
                }

                telemetry.addData("Changing value", "f: " + f);
                telemetry.addData("velocity desired", velocity);
                telemetry.addData("velocity real", shooter.getVelocity());
                telemetry.addData("running?", running);
                telemetry.update();
            }else if(valueChange == 4){

                if(gamepad2.y) {
                    velocity ++;
                }else if(gamepad2.a) {
                    velocity--;
                }
                telemetry.addData("Changing value", "velocity: " + velocity);
                telemetry.addData("velocity desired", velocity);
                telemetry.addData("velocity real", shooter.getVelocity());
                telemetry.addData("running?", running);
                telemetry.update();
            }else if(valueChange == 5){
                valueChange = 0;
            }

            //if(gamepad2.right_stick_button)
            shooter.setVelocityPIDFCoefficients(p,i,d,f);
        }
    }
}
