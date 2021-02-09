package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

public class Shooter {
    public DcMotorEx shooter;
    public Servo kicker;
    public Servo turret;

    public Shooter(DcMotorEx shooterinit, Servo turretinit, Servo kickerinit){
        shooter = shooterinit;
        turret = turretinit;
        kicker = kickerinit;
        turret.setPosition(1);
        kicker.setPosition(1);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setVelocityPIDFCoefficients(.249,2.032,20.835,12.6);

    }

    public void startMotor(double velocity){
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocity(velocity);
    }

    public void stopMotor(){
        shooter.setPower(0);
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
}
