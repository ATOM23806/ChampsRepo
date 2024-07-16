package org.firstinspires.ftc.teamcode.Auton;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.TramRed;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.roadrunner.roadrunner.trajectories.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous (group = "Red Autos", name = "Red Side Far +2")
public class RedSideFarStack extends LinearOpMode {

    private Servo rights, lefts;
    private CRServo intake;

    private VisionPortal visionPortal;
    private TramRed redTram;
    private RevColorSensorV3 colorSensorV3;
    private Servo release, flick;

    private TrajectorySequence finaltraj, stackPath, scorePath;
    Slides slides;

    private Pose2d startingPose;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        startingPose = new Pose2d(-34.71, -62.5, Math.toRadians(90));
        drive.setPoseEstimate(startingPose);
        rights = hardwareMap.get(Servo.class, "rA");
        lefts = hardwareMap.get(Servo.class, "lA");
        intake = hardwareMap.get(CRServo.class, "in");
        release = hardwareMap.get(Servo.class, "release");
        flick = hardwareMap.get(Servo.class, "flick");
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



        TrajectorySequence trajleft = drive.trajectorySequenceBuilder(startingPose)
                .lineTo(new Vector2d(-40.34, -45.76))
                .splineToSplineHeading(new Pose2d(-47.28, -17.98, Math.toRadians(270.00)), Math.toRadians(107.10))
                .addDisplacementMarker(() -> {
                    intake.setPower(-0.5);
                })
                .forward(3.5)
                .back(3.5)
                .splineToSplineHeading(new Pose2d(-33.81, -11.59, Math.toRadians(180.00)), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })
                .lineToConstantHeading(new Vector2d(40.89, -10.48))
                .addDisplacementMarker(120, () -> {
                    System.out.println("Stopping Intake!");
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })
                .splineToConstantHeading(new Vector2d(49.64, -29.92), Math.toRadians(-23.20))
                .back(4)
                .build();



        //TODO: Verify middle
        TrajectorySequence trajCenter = drive.trajectorySequenceBuilder(startingPose)
                .strafeLeft(4)

                .lineToSplineHeading(new Pose2d(-37.09, -13.68, Math.toRadians(270.00)))
                .addDisplacementMarker(() -> {
                    intake.setPower(-0.5);
                })

                .back(6)
                .lineTo(new Vector2d(-35.14, -8.62))
                .lineTo(new Vector2d(-17.95, -8.67))
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })

                .lineTo(new Vector2d(10.60, -8.21))
                .addDisplacementMarker(120, () -> {
                    System.out.println("Stopping Intake!");
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })

                .waitSeconds(5)
                .lineTo(new Vector2d(37.93, -10.60))
                .lineToSplineHeading(new Pose2d(41.69, -17.73, Math.toRadians(180.00)))
                .splineToSplineHeading(new Pose2d(49.82, -28.0, Math.toRadians(180.00)), Math.toRadians(18.44))
                .back(5)

                .build();



        TrajectorySequence trajright = drive.trajectorySequenceBuilder(startingPose)
                .splineToSplineHeading(new Pose2d(-35.55, -31.64, Math.toRadians(0.00)), Math.toRadians(36.27))
                .addDisplacementMarker(() -> {
                    intake.setPower(-0.5);
                })
                .forward(5)
                .back(5)
                .lineTo(new Vector2d(-41.27, -19.48))
                .splineToSplineHeading(new Pose2d(-29.82, -11.52, Math.toRadians(180.00)), Math.toRadians(9.06))
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })
                .lineToConstantHeading(new Vector2d(40.62, -11.32))
                .addDisplacementMarker(120, () -> {
                    System.out.println("Stopping Intake!");
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })
                .splineToConstantHeading(new Vector2d(50.21, -43.27), Math.toRadians(267.69))
                .back(4)
                .build();



        switch (recordedPropPosition) {
            case LEFT:
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
                .forward(13)
                .splineToConstantHeading(new Vector2d(60.06, -11.32), Math.toRadians(-43.15))
                .addDisplacementMarker(() -> {
                    release.setPosition(0);
                })
                .build();

        drive.followTrajectorySequence(fin);


        while(!isStopRequested()) {

        }


    }
}
