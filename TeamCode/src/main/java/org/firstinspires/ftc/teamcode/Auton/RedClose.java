package org.firstinspires.ftc.teamcode.Auton;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.TramRed;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.roadrunner.roadrunner.trajectories.TrajectorySequence;
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
    private TrajectorySequence finaltraj;

    private DcMotor leftlift, rightlift;

    private int runcount = 0 ;
    Slides slides;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(10.1, -59.7, Math.toRadians(90)));
        rights = hardwareMap.get(Servo.class, "rA");
        lefts = hardwareMap.get(Servo.class, "lA");
        intake = hardwareMap.get(CRServo.class, "in");
        release = hardwareMap.get(Servo.class, "release");


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

            while (!isStarted()) {
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
                recordedPropPosition = TramRed.PropPositions.LEFT;
            }


            TrajectorySequence trajleft = drive.trajectorySequenceBuilder(new Pose2d(11.3, -59.7, Math.toRadians(90)))
                    .splineToSplineHeading(new Pose2d(10.00, -30, Math.toRadians(180.00)), Math.toRadians(201.80))
                    .forward(7)
                    .back(4)
                    .addDisplacementMarker(() -> {
                        intake.setPower(0.3);
                    })
                    .back(1)
                    .waitSeconds(.3)
                    .addDisplacementMarker(() -> {
                        intake.setPower(-.3);
                    })
                    .forward(1)
                    .waitSeconds(.2)
                    .back(4)
                    .addDisplacementMarker(() -> {
                        intake.setPower(0);
                    })
                    .lineToConstantHeading(new Vector2d(23.67, -31.99))
                    .addDisplacementMarker(() -> {
                        System.out.println("Stopping Intake!");
                        rights.setPosition(0.47);
                        lefts.setPosition(0.47);
                    })
                    .lineToConstantHeading(new Vector2d(46, -30.40))
                    .back(6,
                            SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                            SampleMecanumDrive.getAccelerationConstraint(40))

                    .build();


            TrajectorySequence trajCenter = drive.trajectorySequenceBuilder(new Pose2d(11.3, -59.7, Math.toRadians(90)))
                    .splineToSplineHeading(new Pose2d(23, -20, Math.toRadians(180.00)), Math.toRadians(90.00))
                    .forward(6)
                    .back(6)
                    .addDisplacementMarker(() -> {
                        intake.setPower(0.3);
                    })
                    .back(1)
                    .waitSeconds(.3)
                    .addDisplacementMarker(() -> {
                        intake.setPower(-.3);
                    })
                    .forward(1)
                    .waitSeconds(.2)
                    .back(4)
                    .addDisplacementMarker(() -> {
                        intake.setPower(0);
                    })


                    .addDisplacementMarker(() -> {
                        System.out.println("Stopping Intake!");
                        rights.setPosition(0.47);
                        lefts.setPosition(0.47);
                    })
                    .lineToSplineHeading(new Pose2d(45, -32.8, Math.toRadians(-180)))
                    .addDisplacementMarker(() -> {

                        intake.setPower(0);
                    })
                    .back(6,
                            SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                            SampleMecanumDrive.getAccelerationConstraint(40))

                    .build();


            TrajectorySequence trajright = drive.trajectorySequenceBuilder(new Pose2d(11.3, -59.7, Math.toRadians(90)))
                    .splineToSplineHeading(new Pose2d(20.08, -35.36, Math.toRadians(90.00)), Math.toRadians(96.01))
                    .forward(6)
                    .back(6)
                    .addDisplacementMarker(() -> {
                        intake.setPower(0.3);
                    })
                    .back(1)
                    .waitSeconds(.3)
                    .addDisplacementMarker(() -> {
                        intake.setPower(-.3);
                    })
                    .forward(1)
                    .waitSeconds(.2)
                    .back(4)
                    .addDisplacementMarker(() -> {
                        intake.setPower(0);
                    })
                    .lineTo(new Vector2d(31.35, -43.84))
                    .addDisplacementMarker(() -> {
                        System.out.println("Stopping Intake!");
                        rights.setPosition(0.47);
                        lefts.setPosition(0.47);
                    })

                    .splineToSplineHeading(new Pose2d(46.81, -41.50, Math.toRadians(180.00)), Math.toRadians(7.08),
                            SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .back(6,
                            SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                            SampleMecanumDrive.getAccelerationConstraint(40))

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
                .lineToConstantHeading(new Vector2d( 43.2,-60))
                .lineToConstantHeading(new Vector2d(60,-60))
                .addDisplacementMarker(() -> {
                    release.setPosition(0);
                })
                .build();

        drive.followTrajectorySequence(fin);


        while(!isStopRequested()) {

        }


    }
}
