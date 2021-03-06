package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class BlinkinOn extends LinearOpMode {
    DcMotor blinkin;

    RevBlinkinLedDriver lights;

    @Override
    public void runOpMode(){
        blinkin = hardwareMap.get(DcMotor.class, "blinkin");

        telemetry.addData("Status", "initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            blinkin.setPower(1);
        }
    }
}
