package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class IntakeMotor extends LinearOpMode {
    public void runOpMode() throws InterruptedException{
        DcMotor intake1 = hardwareMap.get(DcMotor.class, "intake1");
        DcMotor intake2 = hardwareMap.get(DcMotor.class, "intake2");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            intake1.setPower(1);
            intake2.setPower(intake1.getPower());
        }
    }
}
