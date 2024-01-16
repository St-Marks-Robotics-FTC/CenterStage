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
                .setConstraints(55, 40, Math.toRadians(120), Math.toRadians(120), 12)
                .setDimensions(13.5, 14) // Set size of bot
                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(16.5, 63.5, Math.toRadians(90))) // Blue Close Starting Position
                        drive.trajectorySequenceBuilder(new Pose2d(-40, -63.5, Math.toRadians(-90))) // Red Close Starting Position





//                                // Blue close 2+0
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(28, 36, Math.toRadians(-135)), Math.toRadians(-45))
//                                .waitSeconds(1) // Purple Spike
//
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(43, 42, Math.toRadians(180)), Math.toRadians(0))
//                                .waitSeconds(2.5) // Place Yellow
//
//                                .setReversed(false)
//                                .lineToLinearHeading(new Pose2d(46, 58, Math.toRadians(-125))) // Corner Park
////                                .lineToLinearHeading(new Pose2d(43, 13, Math.toRadians(-155))) // Middle Park

                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-31, -31, Math.toRadians(180)), Math.toRadians(45))
                                .waitSeconds(0.5) // Score Purple Spike


                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(-60, -11.5), Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(-60, -11.5, Math.toRadians(180)), Math.toRadians(180))
//                                .waitSeconds(1) // Pick from stack


                                // Drive to Board
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(40, -11.5, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(0.2) // Wat for robot
                                .lineToLinearHeading(new Pose2d(46, -42, Math.toRadians(180)))
                                .waitSeconds(.3)
                                .lineToLinearHeading(new Pose2d(47, -43, Math.toRadians(180)))

                                .waitSeconds(1.5) // Place Yellow


                                // Cycle 2-3
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(23, -11.5), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-60, -11.5), Math.toRadians(180))
                                .waitSeconds(1)

                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(23, -11.5), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(46, -30), Math.toRadians(-45))
                                .waitSeconds(1) // place whites


//                                // 4-5
//                                .setReversed(false)
//                                .setTangent(Math.toRadians(135))
//                                .splineToConstantHeading(new Vector2d(23, -11.5), Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(-60, -11.5), Math.toRadians(180))
//                                .waitSeconds(1)
//
//                                .setReversed(true)
//                                .splineToConstantHeading(new Vector2d(23, -11.5), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(45, -30), Math.toRadians(-45))
//                                .waitSeconds(1) // place whites





//                                .splineToConstantHeading(new Vector2d(43, -42), Math.toRadians(-70))
//                                .splineToSplineHeading(new Pose2d(43, -42, Math.toRadians(180)), Math.toRadians(-90))

//                                .lineToLinearHeading(new Pose2d(43, -13, Math.toRadians(155)))















                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}