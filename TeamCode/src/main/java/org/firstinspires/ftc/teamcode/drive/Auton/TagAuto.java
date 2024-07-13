package org.firstinspires.ftc.teamcode.drive.Auton;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TeleOp.SuperQualsTeleOp;
import org.firstinspires.ftc.teamcode.util.TramRed;
import org.firstinspires.ftc.teamcode.drive.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous (group = "Red Autos", name = "Tagggg")
public class TagAuto extends LinearOpMode {

    private Servo rights, lefts;
    private CRServo intake;
    private SampleMecanumDrive drive;

    private VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    private WebcamName aprilCam;
    public volatile AprilTagDetection desiredTag;
    private boolean USE_WEBCAM = true;
    public volatile boolean targetFound;
    private TagDetectionThread detectionThread;
    public static int DESIRED_TAG_ID = 4;
    private static double DESIRED_DISTANCE = 6;

    public static double SPEED_GAIN = 0.023;
    public static double STRAFE_GAIN = 0.015;
    public static double TURN_GAIN = 0.014;

    public static double MAX_AUTO_SPEED = 0.25;
    public static double MAX_AUTO_STRAFE = 0.25;
    public static double MAX_AUTO_TURN = 0.15;

    public double driv, strafe, turn;

    public volatile Pose2d fcPoseTag;

    FtcDashboard dashboard = FtcDashboard.getInstance();

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
        startingPose = new Pose2d(10.1, -59.7, Math.toRadians(90));
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

        driv = 0;        // Desired forward power/speed (-1 to +1)
        strafe = 0;        // Desired strafe power/speed (-1 to +1)
        turn = 0;        // Desired turning power/speed (-1 to +1)


        initAprilTag();


       /* if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur */


        release.setPosition(0);
        rights.setPosition(0.05);
        lefts.setPosition(0.05);
        detectionThread = new TagDetectionThread(aprilTag);
        detectionThread.start();


        waitForStart();

        targetFound = false;
        desiredTag = null;





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
                .lineToSplineHeading(new Pose2d(27.69, -10.60, Math.toRadians(180.00)))
                //.turn(Math.toRadians(-45))
                .lineTo(new Vector2d(23.6, -28.4))
                .addDisplacementMarker(() -> {
                    System.out.println("Stopping Intake!");
                    rights.setPosition(0.47);
                    lefts.setPosition(0.47);
                })
                .addDisplacementMarker(() -> {
                    synchronized (this) {
                        drive.setPoseEstimate(new Pose2d(fcPoseTag.getX(), fcPoseTag.getY(), drive.getPoseEstimate().getHeading()));
                    }
                })
                .waitSeconds(5)
                .lineToConstantHeading(new Vector2d(49, -40),
                        SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))

                .back(5,
                        SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();

        TrajectorySequence finaltraj = trajleft;

        //drive.followTrajectorySequence(finaltraj);

        while (opModeIsActive()) {

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

                //if (rangeError < 1.5) break;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                driv = -Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = -Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            }
            telemetry.update();

            moveRobot(driv, strafe, turn);
        }


        TrajectorySequence afterTag = drive.trajectorySequenceBuilder(drive.getPoseEstimate()).back(5,
                SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                SampleMecanumDrive.getAccelerationConstraint(40)).build();



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

        aprilCam = hardwareMap.get(WebcamName.class, "Webcam 2");
        ;

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(aprilCam)
                    .addProcessor(aprilTag)
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
