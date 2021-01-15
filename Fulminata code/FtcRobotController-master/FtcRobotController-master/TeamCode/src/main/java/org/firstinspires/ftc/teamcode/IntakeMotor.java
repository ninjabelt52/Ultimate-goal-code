package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class IntakeMotor extends LinearOpMode {
    public void runOpMode() throws InterruptedException{
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        telemetry.addData("Status", "initialized");
        telemetry.update();

        while(opModeIsActive()){
            intake.setPower(1);
        }
    }
}
