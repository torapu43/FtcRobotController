package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        int pos = 1;



        Pose2d startPos = new Pose2d(16.875, 64.4375, Math.toRadians(90));


        RoadRunnerBotEntity blueBoard = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.46)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                .setReversed(true)
                                .splineTo(new Vector2d(15,50),Math.toRadians(-108))
                                .splineTo(new Vector2d(8,40),Math.toRadians(-140))
                            .build());

        RoadRunnerBotEntity blueFar = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel,  maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.46)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-40.875, 64.4375, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d(-56, 32, Math.toRadians(180)))
                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(-36,12),Math.toRadians(180))
                                        .lineToConstantHeading(new Vector2d(-55,12))
                                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(blueBoard)
                .addEntity(blueFar)
                .start();
    }
}