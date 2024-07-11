package org.firstinspires.ftc.teamcode.drive.TeleOp;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.github.i_n_t_robotics.zhonyas.navx.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.drive.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp
public class SuperQualsTeleOp extends LinearOpMode {
    private SampleMecanumDrive drive;
    private CRServo intake;
    private Servo release;
    public volatile double yaw;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    public AprilTagDetection desiredTag;
    public Pose2d currentPose;

    private boolean USE_WEBCAM = true;

    private final Vector2d cameraOffset = new Vector2d(0, 0);


    public static double SPEED_GAIN = 0.03;
    public static double STRAFE_GAIN = 0.015;
    public static double TURN_GAIN = 0.04;
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    public static int DESIRED_TAG_ID = -1;
    public double DESIRED_DISTANCE = 6;

    public boolean targetFound;
    public double driv, strafe, turn;

    enum liftPos {
        ZERO,
        LOW,
        MEDIUM,
        HIGH
    }

    public AHRS navx;

    private Servo rights, lefts;
    boolean normSpeed = true;
    private Servo nahIdMog;
    liftPos pos = liftPos.ZERO;
    int encPos = -20;
    Slides testslides;
    @Override
    public void runOpMode() throws InterruptedException {
        initAprilTag();
        drive = new SampleMecanumDrive(hardwareMap);
        rights = hardwareMap.get(Servo.class, "rA");
        lefts = hardwareMap.get(Servo.class, "lA");
        intake = hardwareMap.get(CRServo.class, "in");
        release = hardwareMap.get(Servo.class, "release");
        nahIdMog = hardwareMap.get(Servo.class,"dr");
        navx =  AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData);

        testslides = new Slides(hardwareMap);
        testslides.resetEnc();
        driv = 0;
        strafe = 0;
        turn = 0;

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        new Thread(new gyroThread()).start();
        waitForStart();
        while (opModeIsActive()) {

            targetFound = false;
            desiredTag = null;

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
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            if (targetFound) {
                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);

                currentPose = cameraToRobotPose(desiredTag);

                Vector2d fcPose = getFCPositionIMU(desiredTag, currentPose, Math.toRadians(yaw));
                Pose2d fcPoseTag = getFCPositionTag(desiredTag, currentPose);

                telemetry.addData("fcX", fcPose.getX());
                telemetry.addData("fcY", fcPose.getY());
                telemetry.addData("IMU Heading", Math.toRadians(yaw));
                telemetry.addData("tag heading", fcPoseTag.getHeading());


            } else {
                telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
            }

            switch(pos)    {
                case ZERO:
                    encPos = -20;
                    break;
                case LOW:
                    encPos =-550;
                    break;
                case MEDIUM:
                    encPos = -1500;
                    break;
                case HIGH:
                    encPos = -2450;
                    break;

            }


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.dpad_left && targetFound) {
                driveToTag();
            } else {
                drive(y, x, rx);
            }

           // drive(y, x, rx);


            if (gamepad1.right_stick_button) {
                navx.zeroYaw();
            }

            testslides.PIDMotion(encPos);

            if(gamepad1.y) {
                rights.setPosition(0.47);
                lefts.setPosition(0.47);
                pos = liftPos.LOW;
                normSpeed = false;
            }
            if(gamepad1.right_trigger == 1.0) {
                rights.setPosition(0.47);
                lefts.setPosition(0.47);
                pos = liftPos.MEDIUM;
                normSpeed = false;
            }
            if(gamepad1.dpad_right){
                rights.setPosition(0.47);
                lefts.setPosition(0.47);
                pos = liftPos.HIGH;
                normSpeed = true;
            }
            if(gamepad1.left_trigger == 1.0) {
                release.setPosition(0);
                rights.setPosition(0.001);
                lefts.setPosition(0.001);
                pos = liftPos.ZERO;
            }
            if(gamepad1.left_bumper) {
                rights.setPosition(0.47);
                lefts.setPosition(0.47);
                pos = liftPos.HIGH;
                normSpeed = false;
            }

            if(gamepad1.a) {
                intake.setPower(1);
            } else if(gamepad1.x) {
                intake.setPower(0);
            } else if(gamepad1.b) {
                intake.setPower(-1);
            } else if(gamepad1.right_bumper) {
                release.setPosition(0.8);
            }

            if(gamepad2.y) {
                nahIdMog.setPosition(0.55);
            }


            drive.update();

            telemetry.addData("x", currentPose.getX());
            telemetry.addData("y", currentPose.getY());
            telemetry.addData("rot", currentPose.getHeading());
            telemetry.update();
        }

    }

    public void driveToTag() {
        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        double headingError = desiredTag.ftcPose.bearing;
        double yawError = desiredTag.ftcPose.yaw;

        driv = -Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        strafe = -Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

        double leftFrontPower = driv - strafe - turn;
        double rightFrontPower = driv + strafe + turn;
        double leftBackPower = driv + strafe - turn;
        double rightBackPower = driv - strafe + turn;

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

    private void    setManualExposure(int exposureMS, int gain) {
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

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    public Pose2d cameraToRobotPose(AprilTagDetection detection) {
        double tagX = detection.ftcPose.x;
        double tagY = detection.ftcPose.y;

        // Adjust for camera offset and orientation
        double robotX = -tagX - 9.0;
        double robotY = -tagY - 0;

        // Assuming your camera is inverted (180 degrees rotated)
        double robotHeading = Math.toRadians(detection.ftcPose.yaw) + Math.PI;  // Rotate by 180 degrees

        return new Pose2d(robotX, robotY, robotHeading);
    }

    //TODO TEST april tag heading vs navxheading
    public Vector2d getFCPositionIMU(AprilTagDetection detection, Pose2d robotPose, double botheading) {
        // get coordinates of the robot in RC coordinates
        double x = robotPose.getX();
        double y = robotPose.getY();

        // invert heading to correct properly + account for camera being flipped 180
        botheading = -botheading + Math.PI;

        // rotate RC coordinates to be field-centric
        double x2 = x * Math.cos(botheading) + y * Math.sin(botheading);
        double y2 = x * -Math.sin(botheading) + y * Math.cos(botheading);
        double absX;
        double absY;

        // add FC coordinates to apriltag position
        //TODO CHECK INVERSIONS
        VectorF tagpose = detection.metadata.fieldPosition;
        if (detection.metadata.id <= 6) { // first 6 are backdrop tags
            absX = -tagpose.get(0) + y2;
            absY = tagpose.get(1) - x2;

        } else { // then just reverse positions
            absX = tagpose.get(0) - y2;
            absY = tagpose.get(1) + x2;
        }
        // Don't send over a pose, as apriltag heading can be off (see discord)
        return new Vector2d(absX, absY);
    }

    private Pose2d getFCPositionTag(AprilTagDetection detection, Pose2d robotPose) {
        double x = robotPose.getX();
        double y = robotPose.getY();
        // invert heading to correct properly
        double botheading = robotPose.getHeading();

        // rotate RC coordinates to be field-centric
        double x2 = x * Math.cos(botheading) + y * Math.sin(botheading);
        double y2 = x * -Math.sin(botheading) + y * Math.cos(botheading);
        double absX;
        double absY;

        // add FC coordinates to apriltag position
        //TODO CHECK INVERSIONS
        VectorF tagpose = detection.metadata.fieldPosition;
        if (detection.metadata.id <= 6) { // first 6 are backdrop tags
            absX = -tagpose.get(0) + y2;
            absY = tagpose.get(1) - x2;

        } else { // then just reverse positions
            absX = tagpose.get(0) - y2;
            absY = tagpose.get(1) + x2;
        }

        return  new Pose2d(absX, absY, botheading);
    }

    public void drive(double y, double x, double rx) {
        double botHeading = -Math.toRadians(yaw);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        drive.leftFront.setPower(frontLeftPower);
        drive.leftRear.setPower(backLeftPower);
        drive.rightFront.setPower(frontRightPower);
        drive.rightRear.setPower(backRightPower);
    }

    public class gyroThread implements Runnable {
        @Override
        public void run() {
            while (!isStopRequested()) {
                synchronized (this) {
                    yaw = navx.getYaw();
                    telemetry.addData("yaw", yaw);
                }
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }
}
