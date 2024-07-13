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

        TrajectorySequence trajCenter = drive.trajectorySequenceBuilder(new Pose2d(52.75, -29, Math.toRadians(180)))
                .splineTo(new Vector2d(32.05, -12.92), Math.toRadians(170.31))
                .splineToSplineHeading(new Pose2d(-6.91, -12.36, Math.toRadians(0.00)), Math.toRadians(180.00))
                .splineTo(new Vector2d(-35.27, -22.00), Math.toRadians(-83.29))
                .splineTo(new Vector2d(-42.81, -40.43), Math.toRadians(168.69))





                .build();

    }
}
