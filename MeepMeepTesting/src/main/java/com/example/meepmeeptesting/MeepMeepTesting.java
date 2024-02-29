package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new  Pose2d(-36.98, 61.77, Math.toRadians(-90)))
                                .splineToSplineHeading(new Pose2d(-35.20, 31.17, Math.toRadians(0.00)), Math.toRadians(-9.16))
                                .forward(3.5)
                                .back(3.5)
                                .lineToSplineHeading(new Pose2d(-42.28, 23.26, Math.toRadians(180.00)))
                                .splineToConstantHeading(new Vector2d(-37.84, 11.18), Math.toRadians(45.00))
                                .lineToConstantHeading(new Vector2d(43.39, 11.18))
                                .splineToConstantHeading(new Vector2d(49.37, 40.48), Math.toRadians(82.23))
                                .back(4)

                                .addDisplacementMarker(125,() -> {
                                    System.out.println("Hello, World!");

                                })

                                .forward(16)
                                .strafeLeft(5)
                                .splineToConstantHeading(new Vector2d(55.68, 5), Math.toRadians(63.43))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}