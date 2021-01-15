package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

public class Shooter {
    public DcMotor shooter;
    public Servo kicker;
    public DcMotor turret;

    public Shooter(DcMotor shooterinit, DcMotor turretinit, Servo kickerinit){
        shooter = shooterinit;
        turret = turretinit;
        kicker = kickerinit;

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void startMotor(){
        shooter.setPower(1);
    }

    public void shoot(int shootTimes){
        for(int i = 0; i < shootTimes; i++) {
            kicker.setPosition(1);
            sleep(10);
            kicker.setPosition(0);
        }
    }

    public void shoot(){
        kicker.setPosition(1);
        sleep(10);
        kicker.setPosition(0);
    }

    public void TurnTurret(int heading){
        heading = Range.clip(heading, 0, 560);

        turret.setTargetPosition(heading);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
    }
}
