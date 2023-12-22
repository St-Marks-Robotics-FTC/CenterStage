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
                .setConstraints(50, 50, Math.toRadians(120), Math.toRadians(120), 15)
                .setDimensions(18, 18) // Set size of bot
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, -60, Math.toRadians(-90))) // Red

                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-31, -34, Math.toRadians(180)), Math.toRadians(35))
                                .waitSeconds(0.5) // place purple


                                .setReversed(false)
                                .setTangent(Math.toRadians(170))
                                .splineToConstantHeading(new Vector2d(-60, -11.5), Math.toRadians(180)) // go to stack
                                .waitSeconds(0.5) // intake stack

                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(24, -11.5, Math.toRadians(180)), Math.toRadians(0))
                                .splineTo(new Vector2d(45, -43), Math.toRadians(0))
                                .waitSeconds(1.5) // place pixels


                                .setReversed(false)
                                .splineTo(new Vector2d(24, -11.5), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-60, -11.5), Math.toRadians(180)) // go to stack
                                .waitSeconds(0.5) // intake stack

                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(15, -11.5, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(38, -11.5, Math.toRadians(150)), Math.toRadians(0))
                                .waitSeconds(0.5) // place pixels



                                .setReversed(false)
                                .setTangent(Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(10, -11.5, Math.toRadians(180)), Math.toRadians(180)) // go to stack
                                .splineToSplineHeading(new Pose2d(-60, -11.5, Math.toRadians(180)), Math.toRadians(180)) // go to stack
                                .waitSeconds(0.5) // intake stack

                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(15, -11.5, Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(38, -11.5, Math.toRadians(150)), Math.toRadians(0))
                                .waitSeconds(0.5) // place pixels










                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}