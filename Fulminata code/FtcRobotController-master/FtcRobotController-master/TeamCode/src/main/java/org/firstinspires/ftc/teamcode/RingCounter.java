//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//public class RingCounter {
//    DistanceSensor bottom, top;
//    private int bottomNum, topNum, totalNum;
//
//    public RingCounter (HardwareMap hardwaremap){
//        bottom = hardwaremap.get(DistanceSensor.class, "bottom");
//        top = hardwaremap.get(DistanceSensor.class, "top");
//    }
//
//    public int numRings(){
//        calculate();
//
//        return topNum;
//    }
//
//    private void calculate(){
//        if(top.getDistance(DistanceUnit.INCH) <){
//            topNum ++;
//        }
//
//        if(bottom.getDistance(DistanceUnit.INCH) <){
//            bottomNum ++;
//        }
//
//
//    }
//}
