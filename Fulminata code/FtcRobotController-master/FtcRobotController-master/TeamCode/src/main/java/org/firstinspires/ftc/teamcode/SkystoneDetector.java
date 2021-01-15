package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetector extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public String wobblePos = "A";
    public double avg;
    public SkystoneDetector() {

    }

    @Override
    public Mat processFrame(Mat input){
        input.copyTo(workingMatrix);

        if(workingMatrix.empty()){
            return input;
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        Mat ringMat = workingMatrix.submat(80, 160, 110, 220);

        Imgproc.rectangle(workingMatrix, new Rect(110, 80, 110, 80), new Scalar(0, 255, 0), 2);

        double ringTotal = Core.sumElems(ringMat).val[2];

        avg = ringTotal/(120*10);


        return workingMatrix;
    }
}
