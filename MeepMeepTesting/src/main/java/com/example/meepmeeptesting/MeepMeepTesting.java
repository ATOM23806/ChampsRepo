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
                                .splineToSplineHeading(new Pose2d(-35.55, -31.64, Math.toRadians(0.00)), Math.toRadians(36.27))

                                .forward(5)
                                .back(5)
                                .lineTo(new Vector2d(-41.27, -19.48))
                                .splineToSplineHeading(new Pose2d(-29.82, -11.52, Math.toRadians(180.00)), Math.toRadians(9.06))

                                .lineToConstantHeading(new Vector2d(40.62, -11.32))

                                .splineToConstantHeading(new Vector2d(50.21, -43.27), Math.toRadians(267.69))
                                .back(4)

                                .forward(13)
                                .splineToConstantHeading(new Vector2d(60.06, -11.32), Math.toRadians(-43.15))





                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}