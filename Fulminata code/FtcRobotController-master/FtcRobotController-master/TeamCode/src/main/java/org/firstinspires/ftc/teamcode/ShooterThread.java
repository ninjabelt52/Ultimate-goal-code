package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ShooterThread implements Runnable{

    private boolean run = false;
    private boolean toggle = false;
    private boolean notUsed = false;

    public static int bottom = 250;

    public int velocity = 1250, bottomThresh = velocity - bottom, topThresh = velocity + 50;
    DcMotorEx shooter1, shooter2;
    PIDFController pid;
    public static double p = 2,i = .6,d = .015,f = 1;

    Telemetry telemetry;

    private Gamepad gamepad2;

    public ShooterThread(HardwareMap hardwareMap){
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        pid = new PIDController(0,0,0);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public ShooterThread(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry){
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        pid = new PIDController(0,0,0);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        gamepad2 = gamepad;
        this.telemetry = telemetry;
    }

    public void startMotor(int velocity){
        this.velocity = velocity;
        bottomThresh = velocity - 150;
        topThresh = velocity + 50;
        //pid.setSetPoint(this.velocity);
    }

    public void stopMotor(){
        run = false;
    }

    public double getVelocity(){
        return shooter1.getVelocity();
    }

    public int getBottomThresh(){
        return bottomThresh;
    }

    public int getTopThresh(){
        return topThresh;
    }

    public boolean isInThresh(){
        return bottomThresh <= getVelocity() && topThresh >= getVelocity();
    }

    @Override
    public void run(){
        while(!Thread.interrupted()) {
            pid.setP(p);
            pid.setI(i);
            pid.setD(d);
            pid.setF(f);
            shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if(gamepad2.b){
                if(!toggle){
                    run = !run;
                    toggle = true;
                }
            }else{
                toggle = false;
            }

            if(gamepad2.left_trigger > 0 && gamepad2.dpad_down){
                if(!notUsed){
                    velocity -= 10;
                    bottomThresh -= 10;
                    topThresh -= 10;
//                    telemetry.speak("velocity" + velocity);
//                    telemetry.update();
                    notUsed = true;
                }
            }else if(gamepad2.dpad_up && gamepad2.left_trigger > 0){
                if(!notUsed){
                    velocity += 10;
                    bottomThresh += 10;
                    topThresh += 10;
//                    telemetry.speak("velocity" + velocity);
//                    telemetry.update();
                    notUsed = true;
                }
            }else if(gamepad2.dpad_up){
                if(!notUsed){
                    velocity = velocity - velocity % 50;
                    velocity += 50;
                    bottomThresh = velocity - bottom;
                    topThresh = velocity + 50;
//                    telemetry.speak("velocity" + velocity);
//                    telemetry.update();
                    notUsed = true;
                }else{}
            }else if(gamepad2.dpad_down){
                if(!notUsed){
                    velocity -= 10;
                    velocity = velocity - velocity % 50;
                    bottomThresh = velocity - bottom;
                    topThresh = velocity + 50;
//                    telemetry.speak("velocity" + velocity);
//                    telemetry.update();
                    notUsed = true;
                }
            }else{
                notUsed = false;
            }

            if (gamepad2.left_bumper) {
                velocity = 1170;
            }

            if (run) {
                shooter1.setVelocity(pid.calculate(shooter1.getVelocity(), velocity));
                shooter2.setPower(shooter1.getPower());
            } else {
                shooter1.setPower(0);
                shooter2.setPower(shooter1.getPower());
            }
        }
    }
}
