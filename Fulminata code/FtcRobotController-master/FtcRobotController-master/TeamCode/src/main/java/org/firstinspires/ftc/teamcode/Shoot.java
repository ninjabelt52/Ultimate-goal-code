package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Shoot implements Runnable{
    public static int sleep = 250;
    private Gamepad gamepad1;

    public static boolean shoot;
    private boolean loop = true;

    private String status = "";

    private Servo kicker;
    public Shoot(HardwareMap hardwareMap, Gamepad gamepad){
        gamepad1 = gamepad;

        kicker = hardwareMap.get(Servo.class, "kicker");
    }

    public void activate(){
        status = "is activated";
        shoot = true;
    }

    public void deactivate(){
        shoot = false;
    }

    public String getStatus(){
        return status;
    }

    @Override
    public void run(){
        while(loop) {
            if (shoot) {
                status = "shoot!" + shoot;
                kicker.setPosition(.6);
                try {
                    Thread.sleep(175);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                kicker.setPosition(.75);
                try {
                    Thread.sleep(175);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else {
                status = "not activated" + shoot;
                kicker.setPosition(.75);
            }
        }
    }
}
