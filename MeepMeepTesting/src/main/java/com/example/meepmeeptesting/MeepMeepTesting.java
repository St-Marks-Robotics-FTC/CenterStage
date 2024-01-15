package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 56, Math.toRadians(120), Math.toRadians(120), 15)
                .setDimensions(13.5, 14) // Set size of bot
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(16.5, 63.5, Math.toRadians(90))) // Blue Close Starting Position




                                // Blue close 2+0
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(28, 36, Math.toRadians(-135)), Math.toRadians(-45))
                                .waitSeconds(1) // Purple Spike

                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(43, 42, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(2.5) // Place Yellow

                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(46, 58, Math.toRadians(-125))) // Corner Park
//                                .lineToLinearHeading(new Pose2d(43, 13, Math.toRadians(-155))) // Middle Park














                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}