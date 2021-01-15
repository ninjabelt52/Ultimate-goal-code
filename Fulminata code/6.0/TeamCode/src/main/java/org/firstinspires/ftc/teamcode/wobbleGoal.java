package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

public class wobbleGoal {

    private Servo claw;
    private Servo rotate;

    public wobbleGoal (Servo clawinit, Servo rotateinit){
        claw = clawinit;
        rotate = rotateinit;

        rotate.setPosition(1);
        claw.setPosition(0);
    }

    public void open(){
        rotate.setPosition(0);
        claw.setPosition(1);
    }

    public void close(){
        claw.setPosition(1);
    }

    public void fold(){
        claw.setPosition(0);
        rotate.setPosition(1);
    }


}
