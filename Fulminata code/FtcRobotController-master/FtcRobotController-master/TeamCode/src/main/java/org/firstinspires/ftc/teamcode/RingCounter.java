package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RingCounter {
    DistanceSensor bottom, top;
    private int bottomNum, topNum, totalNum;
    private boolean topRec = false, bottomRec = true;
    private Servo subtract;
    private Gamepad gamepad1;

    public RingCounter (HardwareMap hardwaremap, Servo subtract, Gamepad gamepad1){
        bottom = hardwaremap.get(DistanceSensor.class, "bottom");
        top = hardwaremap.get(DistanceSensor.class, "top");
        subtract = hardwaremap.get(Servo.class, "kicker");
        this.subtract = subtract;
        this.gamepad1 = gamepad1;
    }

    public int numTopRings(){
        calculate();

        return topNum;
    }

    public int numBottomRings(){
        calculate();

        return bottomNum;
    }

    private void calculate(){
        if(topDist() > 16){
            if(topRec) {
                topNum++;
                topRec = false;
            }
        }else{
            topRec = true;
        }

        if(bottomDist() < 50){
            if(bottomRec) {
                bottomNum++;
                bottomRec = false;
            }
        }else{
            bottomRec = true;
        }

        if(subtract.getPosition() <= .55){
            topNum--;
            bottomNum--;
        }

        if(gamepad1.right_stick_button){
            topNum = 0;
            bottomNum = 0;
        }

    }

    public int topDist(){
        return (int)(top.getDistance(DistanceUnit.INCH) * 10);
    }

    public int bottomDist(){
        return (int)(bottom.getDistance(DistanceUnit.INCH) * 10);
    }
}
