package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class camTest extends LinearOpMode {
    FtcDashboard dash;

    public void runOpMode() throws InterruptedException{
        dash = FtcDashboard.getInstance();
        Telemetry telemetry = dash.getTelemetry();
        CameraView webcam = new CameraView(hardwareMap);
        Thread webcamThread = new Thread(webcam);

        webcamThread.start();
        while(!isStarted()){
            telemetry.addData("Status", webcam.getStatus());
            telemetry.update();
        }
        FtcDashboard.getInstance().startCameraStream(webcam.vuforia, 0);

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Status", webcam.getStatus());
            telemetry.addData("X pos", webcam.returnXCoordinate());
            telemetry.addData("Y pos,", webcam.returnYCoordinate());
            telemetry.addData("rotation", webcam.returnRotation());
            telemetry.addData("rotation2", webcam.returnRotation2());
            telemetry.addData("rotation3", webcam.returnRotation3());
            telemetry.update();
        }
        webcamThread.interrupt();
    }
}
