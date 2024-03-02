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

@Autonomous (group = "Blue Autos", name = "Blue Side Far +2")
public class BlueSideFarStack extends LinearOpMode {

    private Servo rights, lefts;
    private CRServo intake;
    private Servo release, flick;
    private VisionPortal visionPortal;
    private TramBlue blueTram;
    private RevColorSensorV3 colorSensorV3;
    private TrajectorySequence finaltraj, stacktraj, scoretraj;
    Slides slides;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new  Pose2d(-36.98, 61.77, Math.toRadians(-90)));
        rights = hardwareMap.get(Servo.class, "rA");
        lefts = hardwareMap.get(Servo.class, "lA");
        intake = hardwareMap.get(CRServo.class, "in");
        release = hardwareMap.get(Servo.class, "release");
        flick = hardwareMap.get(Servo.class, "flick");
        slides = new Slides(hardwareMap);
        slides.resetEnc();
        release.setPosition(0);
        rights.setPosition(0.20);
        lefts.setPosition(0.20);
        flick.setPosition(0.77);


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
                .splineToSplineHeading(new Pose2d(-35.20, 31.17, Math.toRadians(0.00)), Math.toRadians(-9.16))
                .addDisplacementMarker(() -> {
                    intake.setPower(-0.5);
                })
                .forward(3.5)
                .back(3.5)
                .lineToSplineHeading(new Pose2d(-42.28, 23.26, Math.toRadians(180.00)))
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(-37.84, 11.18), Math.toRadians(45.00))
                .lineToConstantHeading(new Vector2d(43.39, 11.18))
                .addDisplacementMarker(125, () -> {
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })
                .splineToConstantHeading(new Vector2d(49.37, 40.48), Math.toRadians(82.23))
                .back(4)
                .build();

        TrajectorySequence trajLeftStack = drive.trajectorySequenceBuilder(trajleft.end())
                .forward(8)

                .splineToConstantHeading(new Vector2d(29.99, 12.77), Math.toRadians(192.06))
                .splineToConstantHeading(new Vector2d(7.47, 11.64), Math.toRadians(171.87))
                .lineToConstantHeading(new Vector2d(-51.68, 10.84))
                .lineToConstantHeading(new Vector2d(-58.45, 10.45))
                .addDisplacementMarker(() -> {
                    flick.setPosition(.712);
                })

                .back(4)
                .addDisplacementMarker(() -> {
                    flick.setPosition(.77);
                })
                .addDisplacementMarker(() -> {
                    intake.setPower(1);
                })



                .strafeLeft(2)
                .forward(1)

                .build();

        TrajectorySequence trajLeftBoard = drive.trajectorySequenceBuilder(trajLeftStack.end())
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })
                .lineToConstantHeading(new Vector2d(35.36, 11.39))
                .lineToConstantHeading(new Vector2d(46.86, 40.40))
                .build();


        //TODO: Verify middle
        TrajectorySequence trajCenter = drive.trajectorySequenceBuilder(new  Pose2d(-36.98, 61.77, Math.toRadians(-90)))
                .lineTo(new Vector2d(-38.60, 22.18))
                .lineToSplineHeading(new Pose2d(-38.60, 13.31, Math.toRadians(90.00)))
                .addDisplacementMarker(() -> {
                    intake.setPower(-0.3);
                })
                .forward(3)
                .lineToSplineHeading(new Pose2d(-32.18, 11.61, Math.toRadians(180.00)))
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })
                .lineToConstantHeading(new Vector2d(38.41, 11.80))
                .addDisplacementMarker(125, () -> {
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })
                .lineToSplineHeading(new Pose2d(49.35, 34.44, Math.toRadians(180.00)))
                .back(5.5)
                .build();

        TrajectorySequence trajCenterStack = drive.trajectorySequenceBuilder(trajCenter.end())
                .splineToConstantHeading(new Vector2d(29.99, 12.77), Math.toRadians(192.06))
                .splineToConstantHeading(new Vector2d(7.47, 11.64), Math.toRadians(171.87))
                .lineToConstantHeading(new Vector2d(-51.68, 10.84))
                .lineToConstantHeading(new Vector2d(-58.45, 10.45))
                .addDisplacementMarker(() -> {
                    flick.setPosition(.712);
                })
                .back(4)
                .addDisplacementMarker(() -> {
                    flick.setPosition(.77);
                })
                .addDisplacementMarker(() -> {
                    intake.setPower(1);
                })
                .strafeLeft(2)
                .forward(1)

                .build();

        TrajectorySequence trajCenterBoard = drive.trajectorySequenceBuilder(trajCenterStack.end())
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })
                .lineToConstantHeading(new Vector2d(35.36, 11.39))
                .lineToConstantHeading(new Vector2d(46.86, 34.44))
                .back(4)
                .build();




        TrajectorySequence trajright = drive.trajectorySequenceBuilder(new  Pose2d(-36.98, 61.77, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(-51.63, 16.26, Math.toRadians(90.00)), Math.toRadians(-75.71))
                .addDisplacementMarker(() -> {
                    intake.setPower(-0.5);
                })
                .forward(5)
                .back(5)

                .splineToSplineHeading(new Pose2d(-36.14, 9.7, Math.toRadians(180.00)), Math.toRadians(-21.85))
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })
                .lineToConstantHeading(new Vector2d(25.01, 8.8))

                .addDisplacementMarker(() -> {
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })

                .splineToConstantHeading(new Vector2d(46.86, 23.4), Math.toRadians(41.76))

                .back(5)
                .build();

        TrajectorySequence trajRightStack = drive.trajectorySequenceBuilder(trajright.end())
                .splineToConstantHeading(new Vector2d(29.99, 12.77), Math.toRadians(192.06))
                .addDisplacementMarker(() -> {
                    release.setPosition(0);
                })
                .splineToConstantHeading(new Vector2d(7.47, 9.64), Math.toRadians(171.87),
                        SampleMecanumDrive.getVelocityConstraint(50, 50, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(60))



                .lineToConstantHeading(new Vector2d(-51.68, 12),  SampleMecanumDrive.getVelocityConstraint(42, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(50))


                .lineToConstantHeading(new Vector2d(-64.7, 21),
                        SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))

                .addDisplacementMarker(() -> {
                    flick.setPosition(.709);
                })

                .back(.01)
                .waitSeconds(.1)

                .strafeRight(9)
                .addDisplacementMarker(() -> {
                    flick.setPosition(.77);

                })
                .addDisplacementMarker(() -> {
                    intake.setPower(1);
                })
                .strafeLeft(10)
                .forward(3)
                .back(2)
                .forward(2)




                .build();

        TrajectorySequence trajRightBoard = drive.trajectorySequenceBuilder(trajRightStack.end())

                .lineToConstantHeading(new Vector2d(-28.97, 8.3))
                .addDisplacementMarker(() -> {
                    intake.setPower(-1);
                })
                .lineToConstantHeading(new Vector2d(34.88, 8.3))
                .addDisplacementMarker(61,() -> {
                    intake.setPower(0);
                })
                .addDisplacementMarker(65,() -> {
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })
                .splineToConstantHeading(new Vector2d(46.00, 29.00), Math.toRadians(-7.24))

                .back(4.5)

                .build();




        switch (recordedPropPosition) {
            case LEFT:
                finaltraj = trajleft;
                stacktraj = trajLeftStack;
                scoretraj = trajLeftBoard;
                break;
            case MIDDLE:
                finaltraj = trajCenter;
                break;
            case RIGHT:
                finaltraj = trajright;
                stacktraj = trajRightStack;
                scoretraj = trajRightBoard;
                break;
        }

        drive.followTrajectorySequence(finaltraj);
        release.setPosition(0.8);
        try {
            Thread.sleep(800);
        } catch (InterruptedException e){
            System.out.println("Oops fucky wucky");
        };
        rights.setPosition(0.0);
        lefts.setPosition(0.0);
        drive.followTrajectorySequence(stacktraj);
        release.setPosition(0);
       drive.followTrajectorySequence(scoretraj);
       release.setPosition(0.8);
        try {
            Thread.sleep(800);
        } catch (InterruptedException e){
            System.out.println("Oops fucky wucky");
        };

        boolean slidesTop = false;


        rights.setPosition(0.0);
        lefts.setPosition(0.0);
        intake.setPower(0);
        release.setPosition(0.8);
        try {
            Thread.sleep(800);
        } catch (InterruptedException e){
            System.out.println("Oops fucky wucky");
        };



        TrajectorySequence fin = drive.trajectorySequenceBuilder(finaltraj.end())
                .forward(18)
                .strafeLeft(7)
                .splineToConstantHeading(new Vector2d(55.68, 11), Math.toRadians(43.43))
                .addDisplacementMarker(70, () -> {
                    release.setPosition(0);
                })
                .build();

        //drive.followTrajectorySequence(fin);




        while(!isStopRequested()) {

        }


    }
}

/*.lineToConstantHeading(new Vector2d(-35, 9))
                .addDisplacementMarker(() -> {
                    intake.setPower(-1);
                })
                .addDisplacementMarker(70, () -> {
                    intake.setPower(0);
                })


                .addDisplacementMarker(65, () -> {
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(25, 9))

                .lineToConstantHeading(new Vector2d(48.86, 29))

                .back(3.3)
                .addDisplacementMarker(() -> {
                    release.setPosition(0);
                })
*/

