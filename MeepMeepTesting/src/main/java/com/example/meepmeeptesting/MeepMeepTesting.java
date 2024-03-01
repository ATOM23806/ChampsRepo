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
                        drive.trajectorySequenceBuilder(new  Pose2d(-36.98, 61.77, Math.toRadians(180)))
                                .lineTo(new Vector2d(-38.60, 22.18))
                                .lineToSplineHeading(new Pose2d(-38.60, 13.31, Math.toRadians(90.00)))

                                .forward(3)
                                .lineToSplineHeading(new Pose2d(-32.18, 11.61, Math.toRadians(180.00)))

                                .lineToConstantHeading(new Vector2d(38.41, 11.80))

                                .lineToSplineHeading(new Pose2d(49.35, 34.44, Math.toRadians(180.00)))
                                .back(5.5)
                                .forward(8)

                                .splineToConstantHeading(new Vector2d(29.99, 12.77), Math.toRadians(192.06))
                                .splineToConstantHeading(new Vector2d(7.47, 11.64), Math.toRadians(171.87))
                                .lineToConstantHeading(new Vector2d(-51.68, 10.84))
                                .lineToConstantHeading(new Vector2d(-58.45, 10.45))

                                .back(4)



                                .strafeLeft(2)
                                .forward(1)

                                .lineToConstantHeading(new Vector2d(35.36, 11.39))
                                .lineToConstantHeading(new Vector2d(46.86, 34.44))
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