package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static android.os.SystemClock.sleep;

public class Shooter {
    public DcMotorEx shooter1;
    public Servo kicker;
    public Servo turret;
    DistanceSensor bottom, top;
    private int bottomNum, topNum, totalNum;
    private double velocity;
    PIDFController pid = new PIDFController(.25,.2,.01,1);

    public Shooter(DcMotorEx shooterinit, Servo turretinit, Servo kickerinit){
        shooter1 = shooterinit;
        turret = turretinit;
        kicker = kickerinit;
        turret.setPosition(1);
        kicker.setPosition(1);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setVelocityPIDFCoefficients(4.724,.136,.432,12.6);

    }

    public Shooter(HardwareMap hardwareMap){
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        //shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        turret = hardwareMap.get(Servo.class, "turret");
        kicker = hardwareMap.get(Servo.class, "kicker");

        turret.setPosition(1);
        kicker.setPosition(1);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //shooter1.setVelocityPIDFCoefficients(4.724,.136,.432,12.6);
    }

    public void startMotor(double velocity){
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.velocity = velocity;
        pid.setSetPoint(velocity);
        shooter1.setVelocity(pid.calculate(shooter1.getVelocity(),velocity));
        //shooter2.setPower(shooter1.getPower());
    }

    public void stopMotor(){
        shooter1.setPower(0);
        //shooter2.setPower(shooter1.getPower());
    }

    public void shoot(int shootTimes){
        for(int i = 0; i < shootTimes; i++) {
            kicker.setPosition(1);
            sleep(10);
            kicker.setPosition(0);
        }
    }

    public void shoot(){
        kicker.setPosition(.55);
    }

    public void retract(){
        kicker.setPosition(1);
    }

    public void TurnTurret(double heading){
        heading = Range.clip(heading, 0, 1);

        turret.setPosition(heading);
    }

    public boolean isNotThere(){
        return shooter1.getVelocity() < velocity - 50 || shooter1.getVelocity() > velocity + 50;
    }

    public double getVelocity(){
        return shooter1.getVelocity();
    }

//    public double getVelocity(){
//        return shooter.getVelocity();
//    }
//
//    public int numRings(){
//        calculate();
//
//        return topNum;
//    }
//
//    private void calculate(){
//        if(top.getDistance(DistanceUnit.INCH) <){
//            topNum ++;
//        }
//
//        if(bottom.getDistance(DistanceUnit.INCH) <){
//            bottomNum ++;
//        }
//
//
//    }
}
