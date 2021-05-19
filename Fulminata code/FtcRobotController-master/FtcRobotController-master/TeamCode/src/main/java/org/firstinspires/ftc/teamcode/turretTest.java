package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class turretTest extends LinearOpMode {
    public TurretCalculations turret;
    FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException{
//        dash = FtcDashboard.getInstance();
//        Telemetry telemetry = dash.getTelemetry();
        turret = new TurretCalculations(hardwareMap, gamepad2, telemetry, TurretCalculations.Target.BLUE_TOWER_GOAL);
        Thread turretThread = new Thread(turret);
        //FtcDashboard.getInstance().startCameraStream(webcam.vuforia, 0);

        waitForStart();
        turretThread.start();

        while (opModeIsActive()) {
        }
        turretThread.interrupt();
    }
}
