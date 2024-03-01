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
                        drive.trajectorySequenceBuilder(new Pose2d(-34.71, -62.5, Math.toRadians(90)))
                                .lineTo(new Vector2d(-40.34, -45.76))
                                .splineToSplineHeading(new Pose2d(-47.28, -17.98, Math.toRadians(270.00)), Math.toRadians(107.10))

                                .forward(3.5)
                                .back(3.5)
                                .splineToSplineHeading(new Pose2d(-33.81, -11.59, Math.toRadians(180.00)), Math.toRadians(0.00))

                                .lineToConstantHeading(new Vector2d(40.89, -10.48))

                                .splineToConstantHeading(new Vector2d(49.64, -29.92), Math.toRadians(-23.20))
                                .back(4)
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}