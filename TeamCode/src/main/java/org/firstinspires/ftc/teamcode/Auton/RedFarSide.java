package org.firstinspires.ftc.teamcode.Auton;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.github.i_n_t_robotics.zhonyas.navx.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOp.SuperQualsTeleOp;
import org.firstinspires.ftc.teamcode.util.TramRed;
import org.firstinspires.ftc.teamcode.roadrunner.roadrunner.trajectories.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous (group = "Red Autos", name = "Red Side Far")
public class RedFarSide extends LinearOpMode {

    private Servo rights, lefts;
    private CRServo intake;
    private SampleMecanumDrive drive;

    private VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    private WebcamName colorCam, aprilCam;
    public volatile AprilTagDetection desiredTag;
    private TagDetectionThread detectionThread;

    public AHRS navx;

    private boolean USE_WEBCAM = true;
    public volatile boolean targetFound;
    public static int DESIRED_TAG_ID = 6;
    private static double DESIRED_DISTANCE = 8;
    public double rangeError;
    private double fcX, fcY;

    public static double SPEED_GAIN = 0.03;
    public static double STRAFE_GAIN = 0.02;
    public static double TURN_GAIN = 0.015;

    public static double MAX_AUTO_SPEED = 0.7;
    public static double MAX_AUTO_STRAFE = 0.25;
    public static double MAX_AUTO_TURN = 0.4;

    public double driv, strafe, turn;

    public volatile Pose2d fcPoseTag, currentTagPose;
    public double offset;


    FtcDashboard dashboard = FtcDashboard.getInstance();

    private TramRed redTram;
    private RevColorSensorV3 colorSensorV3;
    private Servo release;
    private DcMotor leftlift, rightlift;
    private boolean track = false;


    private Pose2d startingPose;
    public Telemetry supatelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        supatelemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().drawImage("/images/centerstage.jpg", 0, 0, 144, 144);
        //packet.fieldOverlay().drawImage("/dash/powerplay.png", 0, 0, 144, 144);
        dashboard.sendTelemetryPacket(packet);
        drive = new SampleMecanumDrive(hardwareMap);
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
        navx =  AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData);

        offset = Math.toRadians(navx.getFusedHeading());


        driv = 0;        // Desired forward power/speed (-1 to +1)
        strafe = 0;        // Desired strafe power/speed (-1 to +1)
        turn = 0;        // Desired turning power/speed (-1 to +1)

        // main = new SuperQualsTeleOp();

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

        initAprilTag();
        visionPortal.setProcessorEnabled(redTram, true);

       /* if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur */


        release.setPosition(0);
        rights.setPosition(0.05);
        lefts.setPosition(0.05);

        while (!isStarted()) {
            visionPortal.setActiveCamera(colorCam);
            supatelemetry.addData("Currently Recorded Position", redTram.getRecordedPropPosition());
            supatelemetry.addData("Camera State", visionPortal.getCameraState());
            supatelemetry.addData("Currently Detected Mass Center", "x: " + redTram.getLargestContourX() + ", y: " + redTram.getLargestContourY());
            supatelemetry.addData("Currently Detected Mass Area", redTram.getLargestContourArea());
            supatelemetry.addData("Fuck", redTram.getHeight());
            supatelemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        targetFound = false;
        desiredTag = null;


        visionPortal.setProcessorEnabled(redTram, false);
        visionPortal.setProcessorEnabled(aprilTag, true);
        visionPortal.setActiveCamera(aprilCam);


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

                rights.setPosition(0.47);
                lefts.setPosition(0.47);
                })
                .lineTo(new Vector2d(24, -22))
                .turn(Math.toRadians(-20))
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
                .addDisplacementMarker(() -> {
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
                .strafeLeft(3)
                .splineToSplineHeading(new Pose2d(-32.58, -35.16, Math.toRadians(0.00)), Math.toRadians(55.68))
                .addDisplacementMarker(() -> {
                    intake.setPower(0.3);
                })
                .back(3)
                .addDisplacementMarker(() -> {
                    intake.setPower(-0.3);
                })
                .splineToLinearHeading(new Pose2d(-42.97, -15.09, Math.toRadians(180.00)), Math.toRadians(101.31))
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })
                .splineToLinearHeading(new Pose2d(-37.19, -11.54, Math.toRadians(180.00)),
                        Math.toRadians(26.04), SampleMecanumDrive.getVelocityConstraint(50, 40, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .splineToConstantHeading(new Vector2d(-14.92, -11.14), Math.toRadians(3.01), SampleMecanumDrive.getVelocityConstraint(50, 40, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .splineToConstantHeading(new Vector2d(30.83, -11.34), Math.toRadians(-4.27), SampleMecanumDrive.getVelocityConstraint(50, 40, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addDisplacementMarker( () -> {

                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })
                .splineToConstantHeading(new Vector2d(24.40, -27.65), Math.toRadians(259.99))
                .splineToLinearHeading(new Pose2d(38.40, -41.5, Math.toRadians(180.00)), Math.toRadians(-74.59))
                .build();

        TrajectorySequence finaltraj = null;

        switch (recordedPropPosition) {
            case LEFT:
                finaltraj = trajright;
                DESIRED_TAG_ID = 6;
                break;
            case MIDDLE:
                finaltraj = trajright;
                break;
            case RIGHT:
                finaltraj = trajright;
                DESIRED_TAG_ID = 6;
        }

        drive.followTrajectorySequence(finaltraj);

        /*while (opModeIsActive()) {

            desiredTag = detectionThread.getLatestDetection();



            // Tell the driver what we see, and what to do.
            if (desiredTag != null) {
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                if (rangeError < 2) break;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                driv = -Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = -Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            } else if (!targetFound) {
                driv = 0;
                turn = 0;
                strafe = 0;
            }
            telemetry.update();

            moveRobot(driv, strafe, turn);
        } */


        alignLogic(true);

        drive.setPoseEstimate(new Pose2d(fcX, fcY, getCorrectedHeading(Math.toRadians(navx.getFusedHeading()))));
            TrajectorySequence afterTag = drive.trajectorySequenceBuilder(
                    drive.getPoseEstimate())
                    .back(10,
                    SampleMecanumDrive.getVelocityConstraint(20, 30, 9.335),
                    SampleMecanumDrive.getAccelerationConstraint(30))
                    .build();

        drive.followTrajectorySequence(afterTag);


        intake.setPower(0);
        release.setPosition(0.8);


        drive.setPoseEstimate(new Pose2d(51.62, -43.12, Math.toRadians(180.00)));
        TrajectorySequence firstStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(13.68, -8.36), Math.toRadians(180.00), SampleMecanumDrive.getVelocityConstraint(40, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addDisplacementMarker( () -> {
                    rights.setPosition(0);
                    lefts.setPosition(0);
                    release.setPosition(0);
                })
                .splineToConstantHeading(new Vector2d(-37.27, -8.74), Math.toRadians(174.09), SampleMecanumDrive.getVelocityConstraint(40, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addDisplacementMarker( () -> {
                    intake.setPower(1);
                })
                .splineToConstantHeading(new Vector2d(-40.58, -36), Math.toRadians(239.44), SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(30))

                .build();

        drive.followTrajectorySequence(firstStack);

        DESIRED_TAG_ID = 8;
        alignLogic(false);


           // DESIRED_TAG_ID = 8;
            //alignLogic();


       //drive.setPoseEstimate(new Pose2d(fcX, fcY, getCorrectedHeading(Math.toRadians(navx.getFusedHeading()))));
        drive.setPoseEstimate(new Pose2d(-52.4, -35, Math.toRadians(180)));
        TrajectorySequence scoreFirstStack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(8, SampleMecanumDrive.getVelocityConstraint(40, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .back(1)
                .forward(2)
                .back(10)
                .lineToConstantHeading(new Vector2d(-42.85, -16.60)
            ,    SampleMecanumDrive.getVelocityConstraint(40, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addDisplacementMarker( () -> {
                    intake.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(-17.18, -11.20), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(40, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addDisplacementMarker( () -> {
                    rights.setPosition(.47);
                    lefts.setPosition(.47);
                })
                .splineToConstantHeading(new Vector2d(30.59, -14.56), Math.toRadians(-3.58), SampleMecanumDrive.getVelocityConstraint(40, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .splineToConstantHeading(new Vector2d(35.02, -36.58), Math.toRadians(-83.86), SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(20))

                .build();

        drive.followTrajectorySequence(scoreFirstStack);

        int error;
        do {
            error = 380 + rightlift.getCurrentPosition();
            double power = 0.004 * error;
            leftlift.setPower(power);
            rightlift.setPower(power);
            telemetry.addData("pos", rightlift.getCurrentPosition());
            telemetry.update();
        }
        while (Math.abs(error) > 10);

            DESIRED_TAG_ID = 5;
            alignLogic(true);
            drive.setPoseEstimate(new Pose2d(fcX, fcY, getCorrectedHeading(Math.toRadians(navx.getFusedHeading()))));
            drive.followTrajectorySequence(afterTag);

            release.setPosition(.8);

        sleep(2000);


        rights.setPosition(0.0);
        lefts.setPosition(0.0);
        release.setPosition(0);

        requestOpModeStop();




       // drive.followTrajectorySequence(fin);


        while(!isStopRequested()) {
            telemetry.addData("xDDD", drive.getPoseEstimate().getX());
            telemetry.addData("yDDD", drive.getPoseEstimate().getY());

        }


    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(SuperQualsTeleOp.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        colorCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilCam = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(colorCam, aprilCam);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(switchableCamera)
                    .addProcessors(aprilTag, redTram)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private void alignLogic(boolean isBack) {
        while (opModeIsActive()) {
            targetFound = false;
            desiredTag = null;

            if (isBack) {
                visionPortal.setActiveCamera(aprilCam);
            } else {
                visionPortal.setActiveCamera(colorCam);
            }

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);

                if (isBack) {

                    currentTagPose = SuperQualsTeleOp.cameraToRobotPose(desiredTag);

                } else {
                    currentTagPose = SuperQualsTeleOp.frontCcameraToRobotPose(desiredTag);

                }

                fcPoseTag = SuperQualsTeleOp.getFCPositionTag(desiredTag, currentTagPose);

                fcX = fcPoseTag.getX();
                fcY = fcPoseTag.getY();


                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                if (isBack) {
                    rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                } else {
                    rangeError = (desiredTag.ftcPose.range - 10);

                }
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                if (isBack) {
                    if (rangeError < 2) break;
                } else {
                    if (rangeError < 2 && Math.abs(yawError) <= 3) break;
                }

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                if (isBack) {
                    driv = -Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = -Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                } else {
                    driv = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                }


            } else if (!targetFound) {
                driv = 0;
                turn = 0;
                strafe = 0;
            }
            telemetry.update();
            // Apply desired axes motions to the drivetrain.
            moveRobot(driv, strafe, turn);
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        drive.leftFront.setPower(leftFrontPower);
        drive.rightFront.setPower(rightFrontPower);
        drive.leftRear.setPower(leftBackPower);
        drive.rightRear.setPower(rightBackPower);
    }

    public double getCorrectedHeading(double currentHeading) {
        double correctedHeading = offset - currentHeading;
        if (correctedHeading < 0) {
            correctedHeading += 2 * Math.PI;
        } else if (correctedHeading >= 2 * Math.PI) {
            correctedHeading -= 2 * Math.PI;
        }
        return correctedHeading;
    }

    public class TagDetectionThread extends Thread {
        private AprilTagProcessor aprilTag;
        private volatile AprilTagDetection latestDetection;

        public TagDetectionThread(AprilTagProcessor aprilTag) {
            this.aprilTag = aprilTag;
            this.latestDetection = null;
        }


        @Override
        public void run() {
            while (!isInterrupted()) {
                // Update AprilTag detections
                List<AprilTagDetection> detections = aprilTag.getDetections();

                // Find the desired tag or update based on strategy
                AprilTagDetection desiredTag = findDesiredTag(detections);

                // Update the latest detection
                latestDetection = desiredTag;

                // Add a small delay to avoid overloading the CPU
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    interrupt();
                }
            }
        }

        public AprilTagDetection getLatestDetection() {
            return latestDetection;
        }

        private AprilTagDetection findDesiredTag(List<AprilTagDetection> detections) {
            // Implement your logic to find the desired tag
            // Example: Find by ID or other criteria
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null && detection.id == DESIRED_TAG_ID) {
                    return detection;
                }
            }
            return null;
        }
    }
}
