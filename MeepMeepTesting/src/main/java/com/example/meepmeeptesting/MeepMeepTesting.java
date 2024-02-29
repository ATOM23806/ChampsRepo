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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new  Pose2d(-36.98, 61.77, Math.toRadians(90)))
                                .splineToSplineHeading(new Pose2d(-49.63, 16.26, Math.toRadians(90.00)), Math.toRadians(-75.71))
                                .forward(5)
                                .back(5)

                                .splineToSplineHeading(new Pose2d(-36.14, 9.85, Math.toRadians(180.00)), Math.toRadians(-21.85))

                                .lineToConstantHeading(new Vector2d(35.01, 10.36))


                                .waitSeconds(2)
                                .splineToConstantHeading(new Vector2d(46.86, 23.21), Math.toRadians(41.76))

                                .back(4)
                                .forward(16)
                                .splineToConstantHeading(new Vector2d(60.68, 10.85), Math.toRadians(63.43))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}