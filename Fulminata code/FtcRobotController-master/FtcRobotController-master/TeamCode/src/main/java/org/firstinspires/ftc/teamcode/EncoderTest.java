package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class EncoderTest extends LinearOpMode {
    public void runOpMode() {
        DcMotor blw, brw, flw, frw;

        blw = hardwareMap.get(DcMotor.class, "Blw");
        brw = hardwareMap.get(DcMotor.class, "Brw");
        flw = hardwareMap.get(DcMotor.class, "Flw");
        frw = hardwareMap.get(DcMotor.class, "Frw");

        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");

        MecanumDrivetrain drive = new MecanumDrivetrain(blw, brw,flw,frw);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("shooter encoder", shooter.getCurrentPosition());
            telemetry.addLine(drive.TestEncoders());
            telemetry.update();
        }
    }
}
