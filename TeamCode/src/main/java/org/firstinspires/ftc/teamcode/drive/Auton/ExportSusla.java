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

        TrajectorySequence trajCenter = drive.trajectorySequenceBuilder(new Pose2d(-36.98, 61.77, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(-50.63, 16.26, Math.toRadians(90.00)), Math.toRadians(-75.71))

                .forward(5)
                .back(5)

                .splineToSplineHeading(new Pose2d(-36.14, 9.85, Math.toRadians(180.00)), Math.toRadians(-21.85))

                .lineToConstantHeading(new Vector2d(35.01, 9.36))


                .waitSeconds(2)
                .splineToConstantHeading(new Vector2d(46.86, 20.4), Math.toRadians(41.76))

                .back(5)
                .build();
    }
}
