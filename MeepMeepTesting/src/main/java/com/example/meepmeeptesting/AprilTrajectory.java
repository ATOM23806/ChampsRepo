package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AprilTrajectory {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(45.73, -37.56, Math.toRadians(180.00)))
                               /* .strafeLeft(3)
                                .splineToSplineHeading(new Pose2d(-32.58, -35.16, Math.toRadians(0.00)), Math.toRadians(55.68))

                                .back(3)

                                .splineToLinearHeading(new Pose2d(-40.97, -19.09, Math.toRadians(180.00)), Math.toRadians(101.31))

                                .splineToLinearHeading(new Pose2d(-37.19, -11.54, Math.toRadians(180.00)), Math.toRadians(26.04))
                                .splineToConstantHeading(new Vector2d(-14.92, -11.14), Math.toRadians(3.01))
                                .splineToConstantHeading(new Vector2d(30.83, -11.34), Math.toRadians(-4.27))

                                .splineToConstantHeading(new Vector2d(36.40, -27.65), Math.toRadians(259.99))
                                .splineToLinearHeading(new Pose2d(40.40, -41.97, Math.toRadians(180.00)), Math.toRadians(-74.59))
                                .back(10,
                                        SampleMecanumDrive.getVelocityConstraint(20, 30, 9.335),
                                        SampleMecanumDrive.getAccelerationConstraint(30))


                                .splineTo(new Vector2d(32.05, -12.92), Math.toRadians(170.31))
                                .splineTo(new Vector2d(-6.91, -12.36), Math.toRadians(180.00))
                                .splineToConstantHeading(new Vector2d(-35.27, -22.00), Math.toRadians(-83.29))

                                .splineToConstantHeading(new Vector2d(-58.00, -35.00), Math.toRadians(200.35))
                                .forward(3)
                                .back(3)

                                .splineToConstantHeading(new Vector2d(-51.59, -28.54), Math.toRadians(21.71))
                                .splineToConstantHeading(new Vector2d(-29.79, -14.65), Math.toRadians(8.13))
                                .splineToConstantHeading(new Vector2d(4.65, -13.12), Math.toRadians(2.05))
                                .splineToConstantHeading(new Vector2d(36.59, -16.46), Math.toRadians(-34.05))
                                .splineToSplineHeading(new Pose2d(45.73, -37.56, Math.toRadians(180.00)), Math.toRadians(212.47)) */
                                .splineTo(new Vector2d(32.05, -12.92), Math.toRadians(170.31))
                                .splineTo(new Vector2d(-6.91, -12.36), Math.toRadians(180.00))
                                .splineToConstantHeading(new Vector2d(-35.27, -22.00), Math.toRadians(-83.29))

                                .splineToConstantHeading(new Vector2d(-58.00, -35.00), Math.toRadians(200.35))
                                .forward(3)
                                .back(3)

                                .splineToConstantHeading(new Vector2d(-51.59, -28.54), Math.toRadians(21.71))
                                .splineToConstantHeading(new Vector2d(-29.79, -14.65), Math.toRadians(8.13))
                                .splineToConstantHeading(new Vector2d(4.65, -13.12), Math.toRadians(2.05))
                                .splineToConstantHeading(new Vector2d(36.59, -16.46), Math.toRadians(-34.05))
                                .splineToSplineHeading(new Pose2d(42.73, -32.56, Math.toRadians(180.00)), Math.toRadians(212.47))

                                .build()



                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}