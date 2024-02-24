package org.firstinspires.ftc.teamcode.drive.compOpModes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.camera.Tram;
import org.firstinspires.ftc.teamcode.drive.camera.TramBlue;
import org.firstinspires.ftc.teamcode.drive.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
public class BlueSide extends LinearOpMode {

    private Servo rights, lefts;
    private CRServo intake;
    private Servo release;
    private VisionPortal visionPortal;
    private TramBlue redTram;
    private RevColorSensorV3 colorSensorV3;
    Slides slides;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(11.3, 59.7, Math.toRadians(-90)));
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

        redTram = new TramBlue(
                lower,
                upper,
                () -> minArea,
                () -> 410,
                () -> 426
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(redTram)
                .build();

        while(!isStarted()) {
            telemetry.addData("Currently Recorded Position", redTram.getRecordedPropPosition());
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + redTram.getLargestContourX() + ", y: " + redTram.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", redTram.getLargestContourArea());
            telemetry.addData("Fuck", redTram.getHeight());
            telemetry.update();
        }


        waitForStart();
        if (isStopRequested()) return;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        TramBlue.PropPositions recordedPropPosition = redTram.getRecordedPropPosition();

        if (recordedPropPosition == TramBlue.PropPositions.UNFOUND) {
            recordedPropPosition = TramBlue.PropPositions.MIDDLE;
        }



        TrajectorySequence trajleft = drive.trajectorySequenceBuilder(new  Pose2d(11.3, 59.7, Math.toRadians(-90)))
                .splineTo(new Vector2d(5.2,36.6),Math.toRadians(-130))
                .addDisplacementMarker(20,() -> {
                    System.out.println("Fc");
                    intake.setPower(-1);
                })
                .lineToSplineHeading(new Pose2d(46.2,28))
                .turn(Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(55.5,28))
                .addDisplacementMarker(50, () -> {
                    System.out.println("F");
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })
                .build();




        TrajectorySequence trajCenter = drive.trajectorySequenceBuilder(new  Pose2d(11.3, 59.7, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(11.3,32.9))
                .back(2)
                .strafeLeft(2)
                .turn(Math.toRadians(-90))
                .addDisplacementMarker(26.62,() -> {
                    System.out.println("Hello, World!");
                    intake.setPower(-1);
                })
                .lineToConstantHeading(new Vector2d(55,32.9))
                .addDisplacementMarker(40,() -> {
                    System.out.println("Stopping Intake!");
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                    intake.setPower(0);
                })
                .build();


        TrajectorySequence trajright = drive.trajectorySequenceBuilder(new  Pose2d(11.3, 59.7, Math.toRadians(-90)))
                .splineTo(new Vector2d(16.7,37.6),Math.toRadians(-60))
                .addDisplacementMarker(20,() -> {
                    System.out.println("Fc");
                    intake.setPower(-1);
                })
                .back(6)
                .turn(Math.toRadians(60))
                .strafeLeft(6)
                .splineToConstantHeading(new Vector2d(46.7,40),Math.toRadians(0))
                .turn(Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(54.2,40))
                .addDisplacementMarker(50, () -> {
                    System.out.println("Stopping Intake!");
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                    intake.setPower(0);
                })
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
                .lineToConstantHeading(new Vector2d( 43.2,60))
                .lineToConstantHeading(new Vector2d(60,60))
                .build();

        drive.followTrajectorySequence(fin);




        while(!isStopRequested()) {

        }


    }
}
