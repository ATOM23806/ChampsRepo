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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.TramRed;
import org.firstinspires.ftc.teamcode.drive.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous (group = "Red Autos", name = "Red Side Far")
public class RedFarSide extends LinearOpMode {

    private Servo rights, lefts;
    private CRServo intake;

    private VisionPortal visionPortal;
    private TramRed redTram;
    private RevColorSensorV3 colorSensorV3;
    private Servo release;
    private DcMotor leftlift,rightlift;


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

        leftlift = hardwareMap.get(DcMotor.class, "leftlift");
        rightlift = hardwareMap.get(DcMotor.class, "rightlift");
        leftlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftlift.setTargetPosition(0);
        rightlift.setTargetPosition(0);
        leftlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightlift.setDirection(DcMotorSimple.Direction.REVERSE);

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
                .strafeLeft(3)
                .lineTo(new Vector2d(-40.34, -45.76))
                .splineToSplineHeading(new Pose2d(-47, -17.98, Math.toRadians(270.00)), Math.toRadians(107.10))

                .forward(3.5)
                .addDisplacementMarker(() -> {
                    intake.setPower(0.3);
                })
                .back(1)
                .waitSeconds(.6)
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })
                .back(3.5)
                .splineToSplineHeading(new Pose2d(-33.81, -13, Math.toRadians(180.00)), Math.toRadians(0.00),
                        SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))

                .lineToConstantHeading(new Vector2d(30.89, -14.5))
                .addDisplacementMarker( () -> {
                    System.out.println("Stopping Intake!");
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })
                .waitSeconds(5)
                .splineToConstantHeading(new Vector2d(49.64, -31.5), Math.toRadians(-23.20))
                .back(8,
                        SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();



        //TODO: Verify middle
        TrajectorySequence trajCenter = drive.trajectorySequenceBuilder(startingPose)
                .strafeLeft(6)

                .lineToSplineHeading(new Pose2d(-41.09, -12.68, Math.toRadians(270.00)),
                        SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .strafeLeft(2)


                .addDisplacementMarker(() -> {
                    intake.setPower(0.3);
                })
                .back(1)
                .waitSeconds(.6)
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })

                .back(4)

                .lineToSplineHeading(new Pose2d(36.35, -13.18, Math.toRadians(180.00)))
                .addDisplacementMarker( () -> {
                    System.out.println("Stopping Intake!");
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })
                .waitSeconds(7)
                .splineToConstantHeading(new Vector2d(50.81, -31), Math.toRadians(264.81))

                .back(7,
                        SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))

                .build();



        TrajectorySequence trajright = drive.trajectorySequenceBuilder(startingPose)
                .forward(3)
                .strafeLeft(3)
                .splineToSplineHeading(new Pose2d(-33.78, -34.5, Math.toRadians(0.00)), Math.toRadians(-17.11),
                        SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .forward(4)
                .back(5)
                .addDisplacementMarker(() -> {
                    intake.setPower(0.3);
                })
                .back(3)
                .waitSeconds(.4)
                .addDisplacementMarker(() -> {
                    intake.setPower(-.2);
                })
                .forward(1)
                .waitSeconds(.2)
                .back(6)
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })
                .lineToConstantHeading(new Vector2d(-35.75, -10.20))
                .lineToSplineHeading(new Pose2d(41.69, -10.60, Math.toRadians(180.00)))
                .addDisplacementMarker( () -> {
                    System.out.println("Stopping Intake!");
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })
                .waitSeconds(5)
                .lineToConstantHeading(new Vector2d(49, -33.5),
                        SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))

                .back(9,
                        SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();

        TrajectorySequence finaltraj = null;

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








        int error;
        drive.followTrajectorySequence(finaltraj);

        do {
            error = 320 + rightlift.getCurrentPosition();
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
        } catch (InterruptedException e){
            System.out.println("Oops fucky wucky");
        };

        TrajectorySequence fin = drive.trajectorySequenceBuilder(finaltraj.end())
                .forward(10)
                .addDisplacementMarker(() -> {
                    rights.setPosition(0.0);
                    lefts.setPosition(0.0);
                })

                .strafeRight(10)
                .splineToConstantHeading(new Vector2d(55, -13), Math.toRadians(-43.15),
                        SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addDisplacementMarker(() -> {
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
