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
    DcMotorEx shooter1;
    PIDFController pid;
    public static double p = .25,i = .2,d = .01,f = 1;

    public ShooterThread(HardwareMap hardwareMap){
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        //shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        pid = new PIDFController(.25,.2,.01,1);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setPower(0);
        //shooter2.setPower(0);
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
        return shooter1.getVelocity();
    }

    @Override
    public void run(){
        pid.setP(p);
        pid.setI(i);
        pid.setD(d);
        pid.setF(f);
        if(run){
            shooter1.setVelocity(pid.calculate(shooter1.getVelocity(),velocity));
            //shooter1.setPower(1);
            //shooter2.setPower(shooter1.getPower());
        }else {
            shooter1.setVelocity(0);
            //shooter2.setPower(shooter1.getPower());
        }
    }
}
