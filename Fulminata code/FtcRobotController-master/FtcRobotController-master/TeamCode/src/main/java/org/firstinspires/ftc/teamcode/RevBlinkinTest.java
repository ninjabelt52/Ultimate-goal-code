package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RevBlinkinTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        RevBlinkinLedDriver leds;

        leds = hardwareMap.get(RevBlinkinLedDriver.class, "leds");

        waitForStart();
        while(opModeIsActive()){
            //option 1, not exactly what I want, but it does look cool
            //leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);

            //option 2, rotates through red, white and blue with a 350
            //millisecond pause in between
//            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//            sleep(350);
//            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
//            sleep(350);
//            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
//            sleep(350);

            //option 3, same as option 2, but instead it strobes, maybe
            //not? Might give people problems?
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            sleep(1000);
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
            sleep(1000);
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
            sleep(1000);
        }
    }
}
