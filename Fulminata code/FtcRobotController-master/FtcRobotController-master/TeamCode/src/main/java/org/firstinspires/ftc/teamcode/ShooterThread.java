package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ShooterThread implements Runnable{

    private boolean run = false;
    public int velocity, bottomThresh, topThresh;
    DcMotorEx shooter1, shooter2;
    PIDFController pid;
    public static double p = 2,i = .1,d = .01,f = 1;

    public ShooterThread(HardwareMap hardwareMap){
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        pid = new PIDController(0,0,0);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void startMotor(int velocity){
        run = true;
        this.velocity = velocity;
        bottomThresh = velocity - 200;
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
