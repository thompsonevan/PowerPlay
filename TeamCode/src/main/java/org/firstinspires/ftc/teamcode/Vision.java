package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class Vision extends LinearOpMode {
    public static int hVal, sVal, vVal;

    boolean greenDetected = false;

    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();


        while (opModeIsActive()) {
            // tune color values
            if(hVal > 30 && hVal < 100 && sVal > 200 && vVal > 200){ // H : 50-70, S : >245, V : >245
                greenDetected = true;
            } else {
                greenDetected = false;
            }

            telemetry.addData("H", hVal);
            telemetry.addData("S", sVal);
            telemetry.addData("V", vVal);
            telemetry.addData("Green Detected", greenDetected);
            telemetry.update();

            sleep(50);
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        static final Point regionTopLeftAnchor = new Point(109,98);
        static final int REGION_WIDTH = 10;
        static final int REGION_HEIGHT = 10;

        Point regionPointB = new Point(
                regionTopLeftAnchor.x,
                regionTopLeftAnchor.y);
        Point regionPointA = new Point(
                regionTopLeftAnchor.x + REGION_WIDTH,
                regionTopLeftAnchor.y + REGION_HEIGHT);


        Mat regionH, regionS, regionV;
        Mat HSV = new Mat();
        Mat H = new Mat();
        Mat S = new Mat();
        Mat V = new Mat();

        void inputToHSV(Mat input) {
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(HSV, H, 0);
            Core.extractChannel(HSV, S, 1);
            Core.extractChannel(HSV, V, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToHSV(firstFrame);

            regionH = H.submat(new Rect(regionPointA, regionPointB));
            regionS = S.submat(new Rect(regionPointA, regionPointB));
            regionV = V.submat(new Rect(regionPointA, regionPointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToHSV(input);

            hVal = (int) Core.mean(regionH).val[0];
            sVal = (int) Core.mean(regionS).val[0];
            vVal = (int) Core.mean(regionV).val[0];

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