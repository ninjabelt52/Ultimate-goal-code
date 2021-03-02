package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ShooterThread implements Runnable{

    private boolean run;
    private int velocity;
    DcMotorEx shooter;
    PIDFController pid;
    public static double p,i,d,f;

    public ShooterThread(HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        pid = new PIDFController(.25,.2,.01,1);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setPower(0);
    }

    public void startMotor(int velocity){
        run = true;
        this.velocity = velocity;
        pid.setSetPoint(this.velocity);
    }

    public void stopMotor(){
        run = false;
    }

    public double getVelocity(){
        return shooter.getVelocity();
    }

    @Override
    public void run(){
//        pid.setP(p);
//        pid.setI(i);
//        pid.setD(d);
//        pid.setF(f);
        if(run){
            shooter.setVelocity(pid.calculate(shooter.getVelocity(),velocity));
        }else {
            shooter.setVelocity(0);
        }
    }
}
