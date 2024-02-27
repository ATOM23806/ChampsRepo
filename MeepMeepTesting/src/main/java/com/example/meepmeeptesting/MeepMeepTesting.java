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
                        drive.trajectorySequenceBuilder(new  Pose2d(-36.98, -61.77, Math.toRadians(90)))
                                .strafeLeft(4)

                                .lineToSplineHeading(new Pose2d(-43.09, -13.68, Math.toRadians(270.00)))


                                .strafeLeft(3)

                                .back(6)
                                .lineTo(new Vector2d(-35.14, -8.62))
                                .lineTo(new Vector2d(-17.95, -8.67))

                                .lineTo(new Vector2d(10.60, -8.21))

                                .waitSeconds(5)
                                .lineTo(new Vector2d(37.93, -10.60))
                                .lineToSplineHeading(new Pose2d(41.69, -17.73, Math.toRadians(180.00)))
                                .splineToSplineHeading(new Pose2d(49.82, -28.0, Math.toRadians(180.00)), Math.toRadians(18.44))
                                .back(8)
                                .forward(10)
                                .splineToConstantHeading(new Vector2d(61.46, -11.14), Math.toRadians(-1.36))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}