package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@TeleOp
public class OffLight extends LinearOpMode {
    RevColorSensorV3 light;

    public void runOpMode() throws InterruptedException{
        light = hardwareMap.get(RevColorSensorV3.class, "distance");
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        if(light instanceof SwitchableLight) {
            SwitchableLight slight = (SwitchableLight)light;

            slight.enableLight(false);
        }

        waitForStart();

        while(opModeIsActive()){

            light.enableLed(false);
            telemetry.addData("light status1",light.isLightOn());
            telemetry.update();
            sleep(2000);
            light.enableLed(true);
            telemetry.addData("light status",light.isLightOn());
            telemetry.update();
            sleep(2000);
        }
    }
}
