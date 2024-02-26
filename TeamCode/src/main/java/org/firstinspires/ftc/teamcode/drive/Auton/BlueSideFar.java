package org.firstinspires.ftc.teamcode.drive.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.TramBlue;
import org.firstinspires.ftc.teamcode.drive.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous (group = "Blue Autos", name = "Blue Side Far")
public class BlueSideFar extends LinearOpMode {

    private Servo rights, lefts;
    private CRServo intake;
    private Servo release;
    private VisionPortal visionPortal;
    private TramBlue blueTram;
    private RevColorSensorV3 colorSensorV3;
    Slides slides;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new  Pose2d(-36.98, 61.77, Math.toRadians(-90)));
        rights = hardwareMap.get(Servo.class, "rA");
        lefts = hardwareMap.get(Servo.class, "lA");
        intake = hardwareMap.get(CRServo.class, "in");
        release = hardwareMap.get(Servo.class, "release");
        slides = new Slides(hardwareMap);
        slides.resetEnc();
        release.setPosition(0);
        rights.setPosition(0.20);
        lefts.setPosition(0.20);


        Scalar lower = new Scalar(92, 174, 60);
        Scalar upper = new Scalar(178, 255, 251);

        double minArea = 100;

        blueTram = new TramBlue(
                lower,
                upper,
                () -> minArea,
                () -> 250,
                () -> 260
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(blueTram)
                .build();

        while (!isStarted()) {
            telemetry.addData("Currently Recorded Position", blueTram.getRecordedPropPosition());
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + blueTram.getLargestContourX() + ", y: " + blueTram.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", blueTram.getLargestContourArea());
            telemetry.addData("Fuck", blueTram.getHeight());
            telemetry.update();
        }


        waitForStart();
        if (isStopRequested()) return;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        TramBlue.PropPositions recordedPropPosition = blueTram.getRecordedPropPosition();

        if (recordedPropPosition == TramBlue.PropPositions.UNFOUND) {
            recordedPropPosition = TramBlue.PropPositions.MIDDLE;
        }

        TrajectorySequence trajleft = drive.trajectorySequenceBuilder(new  Pose2d(-36.98, 61.77, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(-47.49, 9.84, Math.toRadians(89.18)), Math.toRadians(269.01))
                .build();



        TrajectorySequence trajCenter = drive.trajectorySequenceBuilder(new  Pose2d(-36.98, 61.77, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(-37.09, 13.68, Math.toRadians(90.00)))
                .addDisplacementMarker(() -> {
                    System.out.println("Hello, World!");
                    intake.setPower(-1);
                })
                .back(4)
                .lineTo(new Vector2d(-37.14, 8.62))
                .lineTo(new Vector2d(-17.95, 9.67))
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })
                .lineTo(new Vector2d(10.60, 9.21))
                .addDisplacementMarker(() -> {
                    System.out.println("Stopping Intake!");
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })
                .waitSeconds(5)
                .lineTo(new Vector2d(37.93, 9.60))

                .lineToSplineHeading(new Pose2d(40.20, 15.11, Math.toRadians(180.00)))

                .splineToSplineHeading(new Pose2d(49.25, 26.01, Math.toRadians(180.00)), Math.toRadians(0.00))
                .back(6.5)
                .build();




        TrajectorySequence trajright = drive.trajectorySequenceBuilder(new  Pose2d(-36.98, 61.77, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(-47.49, 9.84, Math.toRadians(89.18)), Math.toRadians(269.01))
                .build();


        TrajectorySequence finaltraj = null;

        switch (recordedPropPosition) {
            case LEFT:
                finaltraj = trajright;
                break;
            case UNFOUND:
                finaltraj = trajCenter;
                break;
            case MIDDLE:
                finaltraj = trajCenter;
                break;
            case RIGHT:
                finaltraj = trajleft;
                break;
            default:
                finaltraj = trajright;
        }



        drive.followTrajectorySequence(finaltraj);

        boolean slidesTop = false;



        intake.setPower(0);
        release.setPosition(0.8);
        try {
            Thread.sleep(800);
        } catch (InterruptedException e){
            System.out.println("Oops fucky wucky");
        };
        rights.setPosition(0.0);
        lefts.setPosition(0.0);


        TrajectorySequence fin = drive.trajectorySequenceBuilder(finaltraj.end())
                .forward(10)
                .lineToConstantHeading(new Vector2d( 52.2,4.5))
                .lineToConstantHeading(new Vector2d(65,4.5))
                .addDisplacementMarker(70, () -> {
                    release.setPosition(0);
                })
                .build();

        drive.followTrajectorySequence(fin);




        while(!isStopRequested()) {

        }


    }
}

/*TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-34.25, 59.54, Math.toRadians(-90.00)))
.lineToSplineHeading(new Pose2d(-37.09, 13.68, Math.toRadians(90.00)))
.lineTo(new Vector2d(-37.14, 8.62))
.lineTo(new Vector2d(-17.95, 9.67))
.lineTo(new Vector2d(10.60, 9.21))
.lineTo(new Vector2d(37.93, 10.60))
.lineToSplineHeading(new Pose2d(46.25, 13.77, Math.toRadians(180.00)))
.splineToConstantHeading(new Vector2d(49.25, 35.01), Math.toRadians(0.00))
.build();
*/
