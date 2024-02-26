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
import org.firstinspires.ftc.teamcode.util.TramRed;
import org.firstinspires.ftc.teamcode.drive.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous (group = "Red Autos", name = "Red Side Close")
public class RedClose extends LinearOpMode {

    private Servo rights, lefts;
    private CRServo intake;

    private VisionPortal visionPortal;
    private TramRed redTram;
    private RevColorSensorV3 colorSensorV3;
    private Servo release;
    Slides slides;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(10.1, -59.7, Math.toRadians(90)));
        rights = hardwareMap.get(Servo.class, "rA");
        lefts = hardwareMap.get(Servo.class, "lA");
        intake = hardwareMap.get(CRServo.class, "in");
        release = hardwareMap.get(Servo.class, "release");
        slides = new Slides(hardwareMap);
        slides.resetEnc();

        Scalar lower = new Scalar(150, 100, 100);
        Scalar upper = new Scalar(180, 255, 255);

        double minArea = 100;

        redTram = new TramRed(
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



        release.setPosition(0);
        rights.setPosition(0.05);
        lefts.setPosition(0.05);

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

        TramRed.PropPositions recordedPropPosition = redTram.getRecordedPropPosition();

        if (recordedPropPosition == TramRed.PropPositions.UNFOUND) {
            recordedPropPosition = TramRed.PropPositions.MIDDLE;
        }



        TrajectorySequence trajleft = drive.trajectorySequenceBuilder(new  Pose2d(11.3, -59.7, Math.toRadians(90)))
                .splineTo(new Vector2d(5.2,-36.6),Math.toRadians(130))
                .addDisplacementMarker(20,() -> {
                    System.out.println("Fc");
                    intake.setPower(-1);
                })
                .lineToSplineHeading(new Pose2d(46.2,-28))
                .turn(Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(54.2,-28))
                .addDisplacementMarker(50, () -> {
                    System.out.println("F");
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })
                .build();




        TrajectorySequence trajCenter = drive.trajectorySequenceBuilder(new  Pose2d(11.3, -59.7, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(11.3,-32.9))
                .turn(Math.toRadians(90))
                .addDisplacementMarker(26.7,() -> {
                    System.out.println("Hello, World!");
                    intake.setPower(-1);
                })
                .lineToConstantHeading(new Vector2d(52.1,-32.9))
                .addDisplacementMarker(40,() -> {
                    System.out.println("Stopping Intake!");
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                    intake.setPower(0);
                })
                .build();


        TrajectorySequence trajright = drive.trajectorySequenceBuilder(new  Pose2d(11.3, -59.7, Math.toRadians(90)))
                .splineTo(new Vector2d(16.7,-37.6),Math.toRadians(60))
                .addDisplacementMarker(20,() -> {
                    intake.setPower(-1);
                    System.out.println("Fc");
                })
                .back(5)
                .turn(Math.toRadians(-60))
                .strafeRight(5)
                .splineToConstantHeading(new Vector2d(46.7,-40),Math.toRadians(0))
                .turn(Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(54.2,-40))
                .addDisplacementMarker(50, () -> {
                    intake.setPower(0);
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                    System.out.println("F");
                })
                .build();
    
        TrajectorySequence finaltraj = null;
        
        switch (recordedPropPosition) {
            case LEFT:
                finaltraj = trajleft;
                break;
            case UNFOUND:
                finaltraj = trajleft;
                break;
            case MIDDLE:
                finaltraj = trajCenter;
                break;
            case RIGHT:
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
                .lineToConstantHeading(new Vector2d( 43.2,-60))
                .lineToConstantHeading(new Vector2d(60,-60))
                .build();

        drive.followTrajectorySequence(fin);


        while(!isStopRequested()) {

        }


    }
}
