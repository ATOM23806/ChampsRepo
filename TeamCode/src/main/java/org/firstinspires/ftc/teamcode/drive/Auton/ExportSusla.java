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
                .lineToSplineHeading(new Pose2d(-39.09, 11.68, Math.toRadians(90.00)))

                .forward(2)
                .lineTo(new Vector2d(-37.14, 8.62))
                .lineTo(new Vector2d(-17.95, 8.67))

                .lineTo(new Vector2d(10.60, 8.61))

                .waitSeconds(5)
                .lineTo(new Vector2d(37.93, 9.60))

                .lineToSplineHeading(new Pose2d(40.20, 15.11, Math.toRadians(180.00)))

                .splineToSplineHeading(new Pose2d(49.25, 26.1, Math.toRadians(180.00)), Math.toRadians(0.00))
                .back(5.5)
                .build();
    }
}
