package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.nio.channels.Pipe;
import java.util.ArrayList;
import java.util.List;

public class CameraView implements Runnable{

    public VuforiaLocalizer vuforia = null;
    OpenCvCamera openCvPassthrough;
    private VuforiaTrackables visionTargets;

    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    private boolean on = true;

    private float robotX = 0;
    private float robotY = 0;
    private float robotAngle = 0;
    private float robotAngle2 = 0;
    private float robotAngle3 = 0;

    private String status = "";

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    private float[] coordinates;

    private int i = 0;
    List<VuforiaTrackable> allTrackables;

    public CameraView(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int [] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[0]);
        parameters.vuforiaLicenseKey = "ASQV2Fr/////AAABmdPYoTImI0MGlZEd8m0rD4xaEgiOXe4RNGM3vbOwB/oE2tBZOB2xznwLB8AOZIcgp7oydcKHdR3zjSWNpfP40tyd2vSdwFeaRIovkpae/9mjz24BGaa+pvsd6LoiAT7HWr4QrAEVJCPxQpgQYUnSrkbigAXx99LU92a+CjQoDodwbWG+hJIqhBdUcxJNpFxbmInXA0n1TpSUVInLQUaaEFyMPveWZtOkflhY93BGgxm2wQrBhwuVbAaOK6znC+fUDecbWXBVNBdbBw44ac4fe1tvZ7DiRuvrTuBr3M5d4l7AlfRnWLbqPkgXQZJ9aXVrO5fZEf1FoIsoVMVYHZ+e3USRbQ3bpoHtQiEAAHDl5G2h";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.useExtendedTracking = false;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        visionTargets = vuforia.loadTrackablesFromAsset("UltimateGoal");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackable blueTowerGoalTarget = visionTargets.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = visionTargets.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = visionTargets.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = visionTargets.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = visionTargets.get(4);
        frontWallTarget.setName("Front Wall Target");

        allTrackables= new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(visionTargets);
        redAllianceTarget.setLocation(createMatrix(0, -halfField, mmTargetHeight, 90, 0, 180));
        blueAllianceTarget.setLocation(createMatrix(0, halfField, mmTargetHeight, 90, 0, 0));
        frontWallTarget.setLocation(createMatrix(-halfField, 0, mmTargetHeight,90, 0, 90));
        blueTowerGoalTarget.setLocation(createMatrix(halfField, quadField, mmTargetHeight,90, 0, -90));
        redTowerGoalTarget.setLocation(createMatrix(halfField, -quadField, mmTargetHeight, 90, 0, -90));

        phoneLocation = createMatrix(-9 * mmPerInch,-2.375f * mmPerInch,3.75f * mmPerInch,0,90,180);

        for(VuforiaTrackable trackable : allTrackables){
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);
        }

        openCvPassthrough = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters, viewportContainerIds[1]);

        openCvPassthrough.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened() {
                openCvPassthrough.setPipeline(new Pipeline(new Scalar(25,100,100,27)));
                openCvPassthrough.startStreaming(0,0, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    class Pipeline extends OpenCvPipeline{
        Scalar color;

        Pipeline(Scalar color)
        {
            this.color = color;
        }

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    color, 4);

            return input;

        }
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x,y,z).multiplied(Orientation.getRotationMatrix(
                AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    @Override
    public void run() {

        lastKnownLocation = createMatrix(0,0,0,0,0,0);
        coordinates = lastKnownLocation.getTranslation().getData();
        visionTargets.activate();

        while(on){

//            if(((VuforiaTrackableDefaultListener)listener).isVisible()) {
            for(VuforiaTrackable trackable : allTrackables){
                while(((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    OpenGLMatrix latestLocation = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();

                    if (latestLocation != null)
                        lastKnownLocation = latestLocation;
                    status = "tracking" + allTrackables.get(i).getName() + ": " + ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() +
                            "\nlast known location: " + formatMatrix(lastKnownLocation);

                    coordinates = lastKnownLocation.getTranslation().getData();


                    robotX = coordinates[0];
                    robotY = coordinates[1];
                    robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                    robotAngle2 = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
                    robotAngle3 = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
                }
            }

            if(i >= 4){
                i = 0;
            }
        }
        visionTargets.deactivate();
    }

    public void turnOff(){
        on = false;
    }

    public void turnOn(){
        on = true;
    }

    public int returnXCoordinate(){
        return (int)robotX;
    }

    public int returnYCoordinate(){
        return (int)robotY;
    }

    public int returnRotation(){
        return (int) robotAngle - 90;
    }

    public int returnRotation2(){
        return (int) robotAngle2;
    }

    public int returnRotation3(){
        return (int) robotAngle3;
    }

    public String getStatus(){
        return status;
    }

    public String formatMatrix(OpenGLMatrix matrix){
        return matrix.formatAsTransform();
    }

}
