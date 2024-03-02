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

        TrajectorySequence trajCenter = drive.trajectorySequenceBuilder(new Pose2d(-64.7, 25, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(29.61, 12.78), Math.toRadians(13.00))
                .lineToConstantHeading(new Vector2d(48.00, 29))
                .addDisplacementMarker(65, () -> {
                    System.out.println("s");
                })





                .build();

    }
}
