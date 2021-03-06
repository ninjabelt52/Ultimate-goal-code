package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/** this is a test Boen made so that I can find the initial values for our pidf controller
 *
 */
@TeleOp
@Disabled
public class MaxVelocity extends LinearOpMode {
    double maxVel = 0;
    double vel = 0;
    DcMotorEx shooter;

    @Override
    public void runOpMode(){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("init");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){
            shooter.setPower(1);
            vel = shooter.getVelocity();

            if(vel > maxVel){
                maxVel = vel;
            }
            telemetry.addData("max velocity", maxVel);
            telemetry.update();
        }
    }
}
