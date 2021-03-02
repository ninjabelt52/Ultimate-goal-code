package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class PIDFTuning extends LinearOpMode {

    public static boolean running = false;
    public static double p = 4.724,i = .136,d = .432, f = 12.6;
    public static int velocity = 2500;
    public static double pos = 1;
    public static boolean shoot = false;
    public static PIDFController pid = new PIDController(0,0,0);

    FtcDashboard dash;
    public void runOpMode(){
        dash = FtcDashboard.getInstance();
        Telemetry Dashboardtelemetry = dash.getTelemetry();
        int valueChange = 0;
        double turretReduction = 0;
        boolean toggle2 = false;
        boolean toggle1 = false;
        double changeValue;
        int lastvelocity = velocity;
        DcMotorEx shooter;
        Servo turret, kicker;

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        turret = hardwareMap.get(Servo.class, "turret");
        kicker = hardwareMap.get(Servo.class, "kicker");

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        pid.setSetPoint(velocity);

        while (opModeIsActive()){

            pid.setP(1);
            pid.setI(1);
            pid.setD(.06);
            pid.setF(1);

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
                    shooter.setVelocity(pid.calculate(shooter.getVelocity(), velocity));

                    if(gamepad2.right_stick_button){
                        lastvelocity = velocity;
                    }
            }else{
                shooter.setPower(0);
            }

            if(gamepad2.right_trigger > 0 && running && shooter.getVelocity() >= 2450 && shooter.getVelocity() <= 2550|| shoot){
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
//                Dashboardtelemetry.addData("Changing value", "p+ " + p);
//                Dashboardtelemetry.addData("velocity", shooter.getVelocity());
//                Dashboardtelemetry.addData("running?", running);
//                Dashboardtelemetry.update();

//                if(gamepad2.y) {
//                    p += changeValue;
//                }else if(gamepad2.a){
//                    p-= changeValue;
//                }
            }else if(valueChange == 1){
//                Dashboardtelemetry.addData("Changing value", "i: "+ i);
//                Dashboardtelemetry.addData("velocity", shooter.getVelocity());
//                Dashboardtelemetry.addData("running?", running);
//                Dashboardtelemetry.update();

//                if(gamepad2.y) {
//                    i += changeValue;
//                }else if(gamepad2.a){
//                    i-= changeValue;
//                }
            }else if(valueChange == 2){
//                Dashboardtelemetry.addData("Changing value", "d: " + d);
//                Dashboardtelemetry.addData("velocity", shooter.getVelocity());
//                Dashboardtelemetry.addData("running?", running);
//                Dashboardtelemetry.update();

//                if(gamepad2.y) {
//                    d += changeValue;
//                }else if(gamepad2.a){
//                    d-= changeValue;
//                }
            }else if(valueChange == 3){


//                if(gamepad2.y) {
//                    f += changeValue;
//                }else if(gamepad2.a) {
//                    f -= changeValue;
//                }

//                Dashboardtelemetry.addData("Changing value", "f: " + f);
//                Dashboardtelemetry.addData("velocity desired", velocity);
//                Dashboardtelemetry.addData("velocity real", shooter.getVelocity());
//                Dashboardtelemetry.addData("running?", running);
//                Dashboardtelemetry.update();
            }else if(valueChange == 4){

                if(gamepad2.y) {
                    velocity ++;
                }else if(gamepad2.a) {
                    velocity--;
                }
//                Dashboardtelemetry.addData("Changing value", "velocity: " + velocity);
//                Dashboardtelemetry.addData("velocity desired", velocity);
//                Dashboardtelemetry.addData("velocity real", shooter.getVelocity());
//                Dashboardtelemetry.addData("running?", running);
//                Dashboardtelemetry.update();
            }else if(valueChange == 5){
                valueChange = 0;
            }

            //if(gamepad2.right_stick_button)
            //shooter.setVelocityPIDFCoefficients(p,i,d,f);

            Dashboardtelemetry.addData("Current Velocity", shooter.getVelocity());
            Dashboardtelemetry.addData("error", pid.getVelocityError());
            Dashboardtelemetry.addData("target Velocity", velocity);
            Dashboardtelemetry.addData("output power", shooter.getPower() * 2580);
            Dashboardtelemetry.update();
        }
    }
}
