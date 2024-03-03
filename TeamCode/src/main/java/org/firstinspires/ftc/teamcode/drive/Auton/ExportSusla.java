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

        TrajectorySequence trajCenter = drive.trajectorySequenceBuilder(new Pose2d(11.3, -59.7, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(11.3,-32.9))
                .turn(Math.toRadians(90))

                .lineToConstantHeading(new Vector2d(52.1,-32.9))





                .build();

    }
}
