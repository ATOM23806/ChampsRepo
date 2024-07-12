package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34.71, -62.5, Math.toRadians(90)))
                                .forward(3)
                                .strafeLeft(3)
                                .splineToSplineHeading(new Pose2d(-33.78, -34.5, Math.toRadians(0.00)), Math.toRadians(-17.11),
                                        SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                                        SampleMecanumDrive.getAccelerationConstraint(40))
                                .forward(4)
                                .back(5)

                                .back(3)
                                .waitSeconds(.4)

                                .forward(1)
                                .waitSeconds(.2)
                                .back(6)

                                .lineToConstantHeading(new Vector2d(-35.75, -10.20))
                                .lineToSplineHeading(new Pose2d(27.69, -10.60, Math.toRadians(180.00)))
                                //.turn(Math.toRadians(-45))
                                .lineTo(new Vector2d(23.6, -28.4))

                                .waitSeconds(1)
                                .lineToConstantHeading(new Vector2d(49, -40),
                                        SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                                        SampleMecanumDrive.getAccelerationConstraint(40))

                                .back(5,
                                        SampleMecanumDrive.getVelocityConstraint(30, 30, 9.335),
                                        SampleMecanumDrive.getAccelerationConstraint(40))
                                .build()



                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}