package org.firstinspires.ftc.teamcode.drive.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class ExportSusla extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence trajCenter = drive.trajectorySequenceBuilder(new Pose2d(49.64, -29.92, Math.toRadians(180.00)))
                .splineToConstantHeading(new Vector2d(33.92, -11.31), Math.toRadians(190.49))
                .lineToConstantHeading(new Vector2d(-42.80, -11.22))
                .lineToConstantHeading(new Vector2d(-58.28, -7.56))
                .build();

    }
}
