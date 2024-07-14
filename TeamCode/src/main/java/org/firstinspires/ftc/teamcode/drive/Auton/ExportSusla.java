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

        TrajectorySequence trajCenter =  drive.trajectorySequenceBuilder(new Pose2d(51, -41.97, Math.toRadians(180.00)))
                .splineTo(new Vector2d(32.05, -12.92), Math.toRadians(170.31))
                .splineTo(new Vector2d(-6.91, -12.36), Math.toRadians(180.00))
                .splineToConstantHeading(new Vector2d(-35.27, -22.00), Math.toRadians(-83.29))
                .splineToConstantHeading(new Vector2d(-58.00, -35.00), Math.toRadians(200.35))


                .forward(3)
                .back(3)






                .build();

    }
}
