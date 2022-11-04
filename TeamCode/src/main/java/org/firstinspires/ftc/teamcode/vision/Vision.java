// Turn this into a Auton common library instead of a teleop class

package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class Vision extends LinearOpMode {
    public static int rVal, gVal, bVal;

    boolean yellowDetected = false;
    boolean greenDetected = false;
    boolean violetDetected = false;

    OpenCvWebcam webcam;

    private int pos = 0;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new SkystoneDeterminationPipeline());

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        waitForStart();

        while (opModeIsActive()) {

            if(gVal > rVal && gVal > bVal && rVal < 200) {
                yellowDetected = false;
                greenDetected = true;
                violetDetected = false;
            } else if (gVal > 200) {
                yellowDetected = true;
                greenDetected = false;
                violetDetected = false;
            } else {
                yellowDetected = false;
                greenDetected = false;
                violetDetected = true;
            }

            telemetry.addData("R", rVal);
            telemetry.addData("G", gVal);
            telemetry.addData("B", bVal);
            telemetry.addData("Yellow Detected", yellowDetected);
            telemetry.addData("Green Detected", greenDetected);
            telemetry.addData("Violet Detected", violetDetected);
            telemetry.update();

            sleep(50);
        }
    }

    public void start(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new SkystoneDeterminationPipeline());

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public int getPos(){

//        if(gVal > rVal && gVal > bVal && rVal < 200) {
//            pos = 0;
//        } else if (gVal > 200) {
//            pos = 1;
//        } else {
//            pos = 2;
//        }

        if(gVal > rVal && gVal > bVal && rVal < 200) {
            pos = 1;
        } else if (gVal > 95) {
            pos = 0;
        } else {
            pos = 2;
    }

//        telemetry.addData("R", rVal);
//        telemetry.addData("G", gVal);
//        telemetry.addData("B", bVal);
//        telemetry.addData("Yellow Detected", yellowDetected);
//        telemetry.addData("Green Detected", greenDetected);
//        telemetry.addData("Violet Detected", violetDetected);
//        telemetry.addData("Pos", pos);
//        telemetry.update();

        sleep(50);

        return pos;
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        static final Point regionTopLeftAnchor = new Point(109,98);
        static final int REGION_WIDTH = 2;
        static final int REGION_HEIGHT = 2;

        Point regionPointB = new Point(
                regionTopLeftAnchor.x,
                regionTopLeftAnchor.y);
        Point regionPointA = new Point(
                regionTopLeftAnchor.x + REGION_WIDTH,
                regionTopLeftAnchor.y + REGION_HEIGHT);


        Mat regionR, regionG, regionB;
        Mat RGB = new Mat();
        Mat R = new Mat();
        Mat G = new Mat();
        Mat B = new Mat();

        void inputToRGB(Mat input) {
            Imgproc.cvtColor(input, RGB, Imgproc.COLOR_RGB2BGR);
            Core.extractChannel(RGB, R, 2);
            Core.extractChannel(RGB, G, 1);
            Core.extractChannel(RGB, B, 0);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToRGB(firstFrame);

            regionR = R.submat(new Rect(regionPointA, regionPointB));
            regionG = G.submat(new Rect(regionPointA, regionPointB));
            regionB = B.submat(new Rect(regionPointA, regionPointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToRGB(input);

            rVal = (int) Core.mean(regionR).val[0];
            gVal = (int) Core.mean(regionG).val[0];
            bVal = (int) Core.mean(regionB).val[0];

            Imgproc.rectangle(
                    input,
                    regionPointA,
                    regionPointB,
                    new Scalar(0, 0, 255),
                    2);

            return input;
        }
    }
}