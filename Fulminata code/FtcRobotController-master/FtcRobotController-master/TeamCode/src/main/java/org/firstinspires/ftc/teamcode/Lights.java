package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lights extends Thread {
    private RevBlinkinLedDriver leds;
    public boolean on = true;
    private boolean loop = true;
    public CustomPattern customPattern;
    private int numTopRings = 0, numBottomRings = 0;
    RingCounter counter;
    Gamepad gamepad1;

    public Lights (HardwareMap hardwareMap, Gamepad gamepad1, Servo subtract){

        leds = hardwareMap.get(RevBlinkinLedDriver.class, "leds");
        counter = new RingCounter(hardwareMap, subtract, gamepad1);

        this.gamepad1 = gamepad1;
    }

    @Override
    public void run (){
        while (loop) {
            if (on) {
                switch (customPattern) {
                    case SPORTHALLE:
//                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//                        try {
//                            Thread.sleep(1000);
//                        } catch (InterruptedException e) {
//                            e.printStackTrace();
//                        }
//                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
//                        try {
//                            Thread.sleep(1000);
//                        } catch (InterruptedException e) {
//                            e.printStackTrace();
//                        }
//                       leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
//                        try {
//                            Thread.sleep(1000);
//                        } catch (InterruptedException e) {
//                            e.printStackTrace();
//                        }

                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);

                    case RINGCOUNTER:
                        if (counter.numTopRings() > numTopRings) {
//                            for (int i = 0; i < counter.numTopRings(); i++) {
//                                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//                                try {
//                                    Thread.sleep(250);
//                                } catch (InterruptedException e) {
//                                    e.printStackTrace();
//                                }
//                                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
//                                try {
//                                    Thread.sleep(100);
//                                } catch (InterruptedException e) {
//                                    e.printStackTrace();
//                                }
//                            }

                            if(counter.numTopRings() == 1){
                                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                            }else if(counter.numTopRings() == 2){
                                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                            }else if(counter.numTopRings() == 3){
                                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                            }
                        }else{
                            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                        }

                        if(counter.numBottomRings() != numBottomRings){
                            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                            try{
                                Thread.sleep(500);
                            }catch (InterruptedException e){
                                e.printStackTrace();
                            }
                            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                            try{
                                Thread.sleep(500);
                            }catch(InterruptedException e){
                                e.printStackTrace();
                            }
                        }else{
                            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                        }
                        numBottomRings = counter.numBottomRings();
                        numTopRings = counter.numTopRings();
                        //on = false;
                    case INIT:
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
                    case ENDGAME:
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
//                        try{
//                            Thread.sleep(1000);
//                        }catch (InterruptedException e){
//                            e.printStackTrace();
//                        }
                        //setPattern(CustomPattern.RINGCOUNTER);
                    default:
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
                }
            } else {
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }
        }
    }

//    public void cancel (){
//        on = false;
//    }
//
//    public void begin (){
//        on = true;
//    }

    public void StopLoop(){
        loop = false;
    }

//    public void RWB(){
//        customPattern = 1;
//    }
//
//    public void rings(int numRings){
//        this.numRings = numRings;
//        on = true;
//        customPattern = 2;
//    }

    public void setPattern(CustomPattern pattern){
        customPattern = pattern;
    }

    public enum CustomPattern{
        SPORTHALLE,
        RINGCOUNTER,
        INIT,
        ENDGAME
    }

    private void on (){
        if(gamepad1.x){
            on = true;
        }else if(gamepad1.a) {
            on = false;
        }else{
            on = true;
        }
    }
}
