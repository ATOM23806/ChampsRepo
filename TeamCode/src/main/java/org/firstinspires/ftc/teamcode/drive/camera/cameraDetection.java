package org.firstinspires.ftc.teamcode.drive.camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class cameraDetection extends LinearOpMode {
    OpenCvWebcam webcam = null;


    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("camerMonitorViewId","id",hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(new examplePipeLine());
        waitForStart();
        while(isStarted()) {
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addLine("Yo mom");
                }
            });
        }
    }

    class examplePipeLine extends OpenCvPipeline {
        Mat HSV = new Mat();
        Mat left,right,center;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(255,0,0);


        @Override
        public Mat processFrame(Mat input) {
            telemetry.addLine("pipeLine Running!");

            Rect leftR = new Rect(1,1,213,359);
            Rect centerR = new Rect(214,1,213,359);
            Rect rightR = new Rect(426,1,213,359);

            input.copyTo(output);
            Imgproc.rectangle(output,leftR,rectColor,2);
            Imgproc.rectangle(output,centerR,rectColor,2);
            Imgproc.rectangle(output,rightR,rectColor,2);

            left = HSV.submat(leftR);
            center = HSV.submat(centerR);
            right = HSV.submat(rightR);


            Core.extractChannel(left, left,0);
            Core.extractChannel(right, right,0);
            Core.extractChannel(center,center,0);

            Scalar leftA = Core.mean(left);
            Scalar rightA = Core.mean(right);
            Scalar centerA = Core.mean(center);

            double leftT = leftA.val[0];
            double rightT = rightA.val[0];
            double centerT = centerA.val[0];

            double test[] = {leftT, rightT, centerT};
            double max = 0;
            for (int i = 1; i < test.length; i++)
                if (test[i] > max)
                    max = test[i];

            if(max == leftT) {
                telemetry.addLine("LEFT");
            } else if(max == rightT) {
                telemetry.addLine("RIGHT");
            } else {
                telemetry.addLine("CENTER");
            }

            return output;

        }
    }
}
