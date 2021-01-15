package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class ShooterMotor extends LinearOpMode {
    public void runOpMode() throws InterruptedException{
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
        telemetry.addData("Status", "initialized");
        telemetry.update();

        while(opModeIsActive()){
            shooter.setPower(1);
        }
    }
}
