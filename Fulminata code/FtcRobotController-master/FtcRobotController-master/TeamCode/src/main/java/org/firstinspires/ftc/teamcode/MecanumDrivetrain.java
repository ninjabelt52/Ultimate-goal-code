package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.os.SystemClock.sleep;

class MecanumDrivetrain {
    public DcMotor m1, m2, m3, m4;
    public BNO055IMU imu;
    public Orientation angles;
    public Orientation lastAngles = new Orientation();
    private double globalAngle;

    public MecanumDrivetrain(DcMotor m1init, DcMotor m2init, DcMotor m3init, DcMotor m4init, BNO055IMU imuinit) {
        m1 = m1init;
        m2 = m2init;
        m3 = m3init;
        m4 = m4init;
        imu = imuinit;

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public MecanumDrivetrain(DcMotor m1init, DcMotor m2init, DcMotor m3init, DcMotor m4init) {
        m1 = m1init;
        m2 = m2init;
        m3 = m3init;
        m4 = m4init;
    }

    public MecanumDrivetrain(HardwareMap hardwareMap){
        m1 = hardwareMap.get(DcMotor.class, "Blw");
        m2 = hardwareMap.get(DcMotor.class, "Flw");
        m3 = hardwareMap.get(DcMotor.class, "Brw");
        m4 = hardwareMap.get(DcMotor.class, "Frw");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu.initialize(parameters);

        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Double gyro(){
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void Drive(double strafe, double straight, double heading, int distance){
        double backleftSpeed, backrightSpeed, frontleftSpeed, frontrightSpeed;


        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double startPosition = m1.getCurrentPosition();
        double frontRPower = straight - strafe;
        double frontLPower = straight + strafe;
        double backRPower = straight + strafe;
        double backLPower = straight - strafe;
        double rampPower = 0;
        boolean rampUp = true;
        boolean rampDown = false;

        while (((m1.getCurrentPosition() < (distance + startPosition)) && (m3.getCurrentPosition() < (distance + startPosition))) ) {
            if((startPosition < m1.getCurrentPosition()) &&(m1.getCurrentPosition() < startPosition + 70)){
                rampUp = true;
                rampDown = false;
            }else if ((((distance + startPosition) - 180) < m1.getCurrentPosition()) && (m1.getCurrentPosition() < (distance + startPosition))) {
                rampDown = true;
                rampUp = false;
            }


            if (rampUp == true){
                rampPower += .15;
                sleep(5);

                if (rampPower >= 1){
                    rampUp = false;
                    rampPower = 1;
                }
            } else if (rampDown) {
                rampPower -= .15;
                sleep(5);
                if (rampPower <= .3){
                    rampDown = ! rampDown;
                    rampPower = .3;
                }
            } else{
                rampPower = 1;
            }



            //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = gyro();

            double correction = (heading - currentHeading) / 360;

            backleftSpeed = (backLPower - (correction * 2)) * rampPower;
            backrightSpeed = (backRPower + (correction * 2)) * rampPower;
            frontleftSpeed = (frontLPower - (correction * 2)) * rampPower;
            frontrightSpeed = (frontRPower + (correction * 2)) * rampPower;

            backrightSpeed = Range.clip(backrightSpeed, -1, 1);
            backleftSpeed = Range.clip(backleftSpeed, -1, 1);
            frontleftSpeed = Range.clip(frontleftSpeed, -1, 1);
            frontrightSpeed = Range.clip(frontrightSpeed, -1, 1);


            m1.setPower(backleftSpeed);
            m2.setPower(frontleftSpeed);
            m3.setPower(backrightSpeed);
            m4.setPower(frontrightSpeed);


            /*telemetry.addData("current heading:", angles.firstAngle);
            telemetry.addData("desired heading:", startPosition);
            telemetry.addData("motor speed:", m1.getPower());*/
        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

        System.out.println("Done");


    }

    public void TurnRight(int desiredHeading){
        double speed;
        double startAngle = gyro();


        while ((Math.abs(gyro()) <= (desiredHeading - 2))) {

            double remainingAngle = -desiredHeading - gyro();
            double dividingRatio = -desiredHeading - startAngle;

            speed = remainingAngle / dividingRatio;


            if (Math.abs(speed) < .2) {
                speed = .2;
            } else {
                speed = remainingAngle / dividingRatio;
            }

            m1.setPower(speed);
            m2.setPower(speed);
            m3.setPower(-speed);
            m4.setPower(-speed);


        }
        Stop();

    }

    public void TurnLeft(int desiredHeading){
        double speed;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startAngle = gyro();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while ((gyro() <= (desiredHeading -  2))){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double remainingAngle = desiredHeading - gyro() ;
            double dividingRatio = desiredHeading - startAngle;

            speed = remainingAngle / dividingRatio;

            if (Math.abs(speed) < .2){
                speed = .2;
            }else {
                speed = remainingAngle / dividingRatio;
            }

            m1.setPower(-speed);
            m2.setPower(-speed);
            m3.setPower(speed);
            m4.setPower(speed);

        }
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

    }

    /**
     * @param strafe power in the strafe direction, double, must be less than or equal to 0
     * @param straight power in the forwards direction, double, must be less than or equal to 0
     * @param heading heading the robot will be facing, make sure to make it the heading you are currently at, or your distance may be inaccurate
     * @param distance distance in clicks the robot will move, must be less than 0 in order to move
     */
    public void Backup(double strafe, double straight, double heading, int distance){
        double backleftSpeed, backrightSpeed, frontleftSpeed, frontrightSpeed;


        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double startPosition = m1.getCurrentPosition();
        double frontRPower = straight - strafe;
        double frontLPower = straight + strafe;
        double backRPower = straight + strafe;
        double backLPower = straight - strafe;
        double rampPower = 0;
        boolean rampUp = true;
        boolean rampDown = false;

        while (((m1.getCurrentPosition() > (distance + startPosition)) && (m3.getCurrentPosition() > (distance + startPosition)))) {
            if((startPosition < m1.getCurrentPosition()) &&(m1.getCurrentPosition() < startPosition + 70)){
                rampUp = true;
                rampDown = false;
            }else if ((((distance + startPosition) - 180) < m1.getCurrentPosition()) && (m1.getCurrentPosition() < (distance + startPosition))){
                rampDown = true;
                rampUp = false;
            }


            if (rampUp == true){
                rampPower += .15;
                sleep(5);

                if (rampPower >= 1){
                    rampUp = false;
                    rampPower = 1;
                }
            } else if (rampDown) {
                rampPower -= .15;
                sleep(5);
                if (rampPower <= .3){
                    rampDown = ! rampDown;
                    rampPower = .3;
                }
            } else{
                rampPower = 1;
            }



            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currentHeading = gyro();

            double correction = (heading - currentHeading) / 100;

            backleftSpeed = backLPower - (correction * 2);
            backrightSpeed = backRPower + (correction * 2);
            frontleftSpeed = frontLPower - (correction * 2);
            frontrightSpeed = frontRPower + (correction * 2);

            backrightSpeed = Range.clip(backrightSpeed, -1, 1);
            backleftSpeed = Range.clip(backleftSpeed, -1, 1);
            frontleftSpeed = Range.clip(frontleftSpeed, -1, 1);
            frontrightSpeed = Range.clip(frontrightSpeed, -1, 1);


            m1.setPower(backleftSpeed * rampPower);
            m2.setPower(frontleftSpeed * rampPower);
            m3.setPower(backrightSpeed * rampPower);
            m4.setPower(frontrightSpeed * rampPower);


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            /*telemetry.addData("current heading:", angles.firstAngle);
            telemetry.addData("desired heading:", startPosition);
            telemetry.addData("motor speed:", m1.getPower());*/
        }

        Stop();


    }

    public String TestEncoders(){
        return "m1 encoder pos: " + m1.getCurrentPosition() + "\nm2 encoder pos: " + m2.getCurrentPosition() + "\nm3 encoder pos: " + m3.getCurrentPosition() + "\nm4 encoder pos: " + m4.getCurrentPosition();
    }

    public void Stop(){
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }

}


