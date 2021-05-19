package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class DriveThread implements Runnable{
    private DcMotor Blw, Brw, Flw, Frw;
    private double straight, strafe, rotate;
    private Gamepad gamepad1;
    private double speedReduce = 1;

    private boolean loop = true;
    public DriveThread(HardwareMap hardwareMap, Gamepad gamepad){
        Blw = hardwareMap.get(DcMotor.class, "Blw");
        Brw = hardwareMap.get(DcMotor.class, "Brw");
        Flw = hardwareMap.get(DcMotor.class, "Flw");
        Frw = hardwareMap.get(DcMotor.class, "Frw");

        Blw.setDirection(DcMotor.Direction.REVERSE);
        Flw.setDirection(DcMotor.Direction.REVERSE);

        Brw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Blw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Frw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Flw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gamepad1 = gamepad;
    }

    public void run(){
        while(loop) {
            straight = gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

            //reduce the speed if so desired
            if(gamepad1.right_trigger > 0){
                speedReduce = .5;
            } else {
                speedReduce = 1;
            }

            Blw.setPower((straight - strafe + rotate) * speedReduce);
            Brw.setPower((straight + strafe - rotate) * speedReduce);
            Flw.setPower((straight + strafe + rotate) * speedReduce);
            Frw.setPower((straight - strafe - rotate) * speedReduce);
        }
    }
}
