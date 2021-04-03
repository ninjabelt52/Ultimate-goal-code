package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class KnockDownArm extends Thread{
    private Servo arm;
    private boolean run = true;
    private Gamepad gamepad1;
    private boolean toggle;
    public KnockDownArm(HardwareMap hardwareMap, Gamepad gamepad){
        arm = hardwareMap.get(Servo.class, "KnockDownArm");
        gamepad1 = gamepad;

        arm.setPosition(0);
    }

    @Override
    public void run(){
        run = true;
        while(run){
//            if(gamepad1.x){
//                if(!toggle){
//                    run = !run;
//                    toggle = true;
//                }else{}
//            }else{
//                toggle = false;
//            }
//
//            arm.setPosition(.85);
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//            arm.setPosition(0);
//
//            try {
//                Thread.sleep(1000);
//            }catch (InterruptedException e){
//                e.printStackTrace();
//            }

            if(gamepad1.x){
                arm.setPosition(.8);
            }else{
                arm.setPosition(0);
            }
        }
    }

    public void cancel(){
        run = false;
    }
}
