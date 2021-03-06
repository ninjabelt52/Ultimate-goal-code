package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
@Disabled
public class backupTest extends LinearOpMode {
    public void runOpMode(){
        DcMotor blw,brw,flw,frw;
        BNO055IMU imu;

        blw = hardwareMap.get(DcMotor.class, "Blw");
        brw = hardwareMap.get(DcMotor.class, "Brw");
        flw = hardwareMap.get(DcMotor.class, "Flw");
        frw = hardwareMap.get(DcMotor.class, "Frw");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu.initialize(parameters);

        blw.setDirection(DcMotorSimple.Direction.REVERSE);
        flw.setDirection(DcMotorSimple.Direction.REVERSE);

        MecanumDrivetrain Drive = new MecanumDrivetrain(blw, flw, brw, frw, imu);

        while(!isStarted()) {
            telemetry.addData("gyro", Drive.gyro());
            telemetry.update();
        }
        waitForStart();
        Drive.TurnRight(180);
        Drive.Backup(-.25,0,-180,-1120);
    }
}
