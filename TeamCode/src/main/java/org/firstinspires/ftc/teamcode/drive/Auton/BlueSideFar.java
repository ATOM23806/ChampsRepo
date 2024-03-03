package org.firstinspires.ftc.teamcode.drive.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.FarTram;
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
    private FarTram blueTram;
    private RevColorSensorV3 colorSensorV3;
    private TrajectorySequence finaltraj;
    private Pose2d startingPose;
    private DcMotor leftlift,rightlift;
    private ElapsedTime timer;



    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        startingPose = new  Pose2d(-39.98, 61.77, Math.toRadians(-90));
        drive.setPoseEstimate(startingPose);
        rights = hardwareMap.get(Servo.class, "rA");
        lefts = hardwareMap.get(Servo.class, "lA");
        intake = hardwareMap.get(CRServo.class, "in");
        release = hardwareMap.get(Servo.class, "release");

        release.setPosition(0);
        rights.setPosition(0.05);
        lefts.setPosition(0.05);
        timer = new ElapsedTime();

        leftlift = hardwareMap.get(DcMotor.class, "leftlift");
        rightlift = hardwareMap.get(DcMotor.class, "rightlift");
        leftlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftlift.setTargetPosition(0);
        rightlift.setTargetPosition(0);
        leftlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightlift.setDirection(DcMotorSimple.Direction.REVERSE);


        Scalar lower = new Scalar(92, 174, 60);
        Scalar upper = new Scalar(178, 255, 251);

        double minArea = 100;
        if ( true ) {

            blueTram = new FarTram(
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

            FarTram.PropPositions recordedPropPosition = blueTram.getRecordedPropPosition();

            if (recordedPropPosition == FarTram.PropPositions.UNFOUND) {
                recordedPropPosition = FarTram.PropPositions.MIDDLE;
            }

            TrajectorySequence trajleft = drive.trajectorySequenceBuilder(startingPose)
                    .splineToSplineHeading(new Pose2d(-37.20, 32.6, Math.toRadians(0.00)), Math.toRadians(-9.16),
                            SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .forward(4)
                    .back(4)
                    .addDisplacementMarker(() -> {
                        intake.setPower(0.3);
                    })
                    .back(1)
                    .waitSeconds(.4)
                    .addDisplacementMarker(() -> {
                        intake.setPower(-.3);
                    })
                    .forward(1)
                    .waitSeconds(.2)
                    .back(4)
                    .addDisplacementMarker(() -> {
                        intake.setPower(0);
                    })
                    .lineToSplineHeading(new Pose2d(-42.28, 23.26, Math.toRadians(180.00)))

                    .splineToConstantHeading(new Vector2d(-37.84, 9.7), Math.toRadians(45.00))
                    .lineToConstantHeading(new Vector2d(40.39, 9.7))
                    .waitSeconds(1)
                    .addDisplacementMarker(125, () -> {
                        rights.setPosition(0.47);
                        lefts.setPosition(0.47);
                    })
                    .splineToConstantHeading(new Vector2d(49.37, 37.48), Math.toRadians(82.23),
                            SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .back(6,
                            SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .build();


            //TODO: Verify middle
            TrajectorySequence trajCenter = drive.trajectorySequenceBuilder(startingPose)
                    .lineTo(new Vector2d(-41.60, 13.18))
                    .lineToSplineHeading(new Pose2d(-41.60, 12, Math.toRadians(90.00)))
                    .addDisplacementMarker(() -> {
                        intake.setPower(0.3);
                    })
                    .back(1)
                    .waitSeconds(.6)
                    .addDisplacementMarker(() -> {
                        intake.setPower(0);
                    })
                    .back(2)

                    .lineToSplineHeading(new Pose2d(-32.18, 9, Math.toRadians(180.00)))

                    .lineToConstantHeading(new Vector2d(38.41, 9.80))
                    .addDisplacementMarker(() -> {
                        rights.setPosition(0.47);
                        lefts.setPosition(0.47);
                    })
                    .waitSeconds(1)

                    .lineToSplineHeading(new Pose2d(50, 26.5, Math.toRadians(180.00)),
                            SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .back(8,
                            SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .build();


            TrajectorySequence trajright = drive.trajectorySequenceBuilder(startingPose)
                    .splineToSplineHeading(new Pose2d(-48, 16.26, Math.toRadians(90.00)), Math.toRadians(-75.71))
                    .forward(4)
                    .back(4)
                    .addDisplacementMarker(() -> {
                        intake.setPower(0.3);
                    })
                    .back(1)
                    .waitSeconds(.4)
                    .addDisplacementMarker(() -> {
                        intake.setPower(-.3);
                    })
                    .forward(1)
                    .waitSeconds(.2)
                    .back(4)
                    .addDisplacementMarker(() -> {
                        intake.setPower(0);
                    })

                    .splineToSplineHeading(new Pose2d(-36.14, 8.7, Math.toRadians(180.00)), Math.toRadians(-21.85))
                    .addDisplacementMarker(() -> {
                        intake.setPower(0);
                    })
                    .lineToConstantHeading(new Vector2d(35.01, 8.7))

                    .addDisplacementMarker(() -> {
                        rights.setPosition(0.47);
                        lefts.setPosition(0.47);
                    })
                    .waitSeconds(2)
                    .splineToConstantHeading(new Vector2d(48, 22), Math.toRadians(41.76))

                    .back(8,
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
                    break;
            }
        }

            int error;
            drive.followTrajectorySequence(finaltraj);
            do {
                error = 380 + rightlift.getCurrentPosition();
                double power = 0.004 * error;
                leftlift.setPower(power);
                rightlift.setPower(power);
                telemetry.addData("pos", rightlift.getCurrentPosition());
                telemetry.update();
            }
                while (Math.abs(error) > 10);
            leftlift.setPower(0);
            rightlift.setPower(0);

            intake.setPower(0);
            release.setPosition(0.8);
            try {
                Thread.sleep(800);
            } catch (InterruptedException e) {
                System.out.println("Oops fucky wucky");
            }

            TrajectorySequence fin = drive.trajectorySequenceBuilder(finaltraj.end())
                    .forward(18)
                    .strafeLeft(10)
                    .addDisplacementMarker(70, () -> {
                        rights.setPosition(0.0);
                        lefts.setPosition(0.0);
                    })

                    .splineToConstantHeading(new Vector2d(55.68, 8), Math.toRadians(43.43),
                            SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .addDisplacementMarker(70, () -> {
                        release.setPosition(0);
                    })
                    .build();

            drive.followTrajectorySequence(fin);



        do {
            error = 0 + rightlift.getCurrentPosition();
            double power = 0.004 * error ;
            leftlift.setPower(power);
            rightlift.setPower(power);
            telemetry.addData("zlpos", leftlift.getCurrentPosition());
            telemetry.addData("zrpos", rightlift.getCurrentPosition());
            telemetry.update() ;
        }
            while ( Math.abs(error) > 20) ;

        leftlift.setPower(0);
        rightlift.setPower(0);


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

