package org.firstinspires.ftc.teamcode.drive.TeleOp;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp
public class SuperQualsTeleOp extends LinearOpMode {
    private SampleMecanumDrive drive;
    private CRServo intake;
    private Servo release;
    enum liftPos {
        ZERO,
        LOW,
        MEDIUM,
        HIGH
    }

    private Servo rights, lefts;
    boolean normSpeed = true;
    private Servo nahIdMog;
    liftPos pos = liftPos.ZERO;
    int encPos = -20;
    Slides testslides;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        rights = hardwareMap.get(Servo.class, "rA");
        lefts = hardwareMap.get(Servo.class, "lA");
        intake = hardwareMap.get(CRServo.class, "in");
        release = hardwareMap.get(Servo.class, "release");
        nahIdMog = hardwareMap.get(Servo.class,"dr");
        testslides = new Slides(hardwareMap);
        testslides.resetEnc();

        waitForStart();
        while (opModeIsActive()) {

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


            if(pos == liftPos.ZERO || normSpeed) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y/2.5,
                                -gamepad1.left_stick_x/2.5,
                                -gamepad1.right_stick_x/2.5
                        )
                );
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
        }

    }
}
