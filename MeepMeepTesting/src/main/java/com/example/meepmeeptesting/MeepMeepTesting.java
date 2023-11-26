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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.46)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(16.875, 64.4375, 90))
                                .setReversed(true)
                                .splineTo(new Vector2d(5,30),Math.toRadians(210))
                                .setReversed(false)
                                .forward(6)
                                .lineToLinearHeading(new Pose2d(48,30,Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-12,60),Math.toRadians(180))
                                .lineToConstantHeading(new Vector2d(-12,12))
                                .lineToConstantHeading(new Vector2d(-55,12))
                                .forward(6)
                                .back(7)
                                .splineToConstantHeading(new Vector2d(-12,12),Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(36,36),Math.toRadians(90))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}