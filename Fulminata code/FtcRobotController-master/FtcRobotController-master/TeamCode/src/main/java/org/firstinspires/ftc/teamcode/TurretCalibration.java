package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class TurretCalibration extends LinearOpMode {
    public static double minRange = .25;
    public static double maxRange = .98;
    public static double setPos = 0;

    public void runOpMode() throws InterruptedException{
        Servo turret = hardwareMap.get(Servo.class, "turret");

        turret.scaleRange(minRange, maxRange);
        waitForStart();
        while (opModeIsActive()) {
            turret.scaleRange(minRange, maxRange);
            turret.setPosition(setPos);
        }
    }
}
