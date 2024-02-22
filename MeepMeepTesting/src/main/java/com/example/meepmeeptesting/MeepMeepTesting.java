package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 45, Math.toRadians(145), Math.toRadians(120), 15)
                .setDimensions(13.5, 14) // Set size of bot
                .followTrajectorySequence(drive ->
                        //drive.trajectorySequenceBuilder(new Pose2d(-41, 63.5, Math.toRadians(-90))) // Blue Far
                        drive.trajectorySequenceBuilder(new Pose2d(16.5, 63.5, Math.toRadians(-90))) // Close


//                                .splineToSplineHeading(new Pose2d(-34, -32, Math.toRadians(180)), Math.toRadians(35))
//                                // place purple from back

//                                .setReversed(true)
////                                .splineTo(new Vector2d(-31, -34), Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(-30, -36, Math.toRadians(-135)), Math.toRadians(45))
//                                .setReversed(false)
//                                .setReversed(true)
//                                .setTangent(Math.toRadians(90))
//                                .splineToSplineHeading(new Pose2d(-42, -34, Math.toRadians(-60)), Math.toRadians(90))

                        //drive.trajectorySequenceBuilder(new Pose2d(17, 63.5, Math.toRadians(-90))) // Blue Close
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(16.5, -32, Math.toRadians(-90)), Math.toRadians(90))
                                //close
//                                .setReversed(true)
//                                .setTangent(Math.toRadians(-90))
//                                .splineToSplineHeading(new Pose2d(23, 42, Math.toRadians(90)), Math.toRadians(-90))
//                                .waitSeconds(2)
//                                .setTangent(Math.toRadians(15))
//                                //.splineToConstantHeading(new Vector2d(11, 32), Math.toRadians(-80))
//                                .splineToSplineHeading(new Pose2d(48, 30, Math.toRadians(180)), Math.toRadians(-30))
//                                .waitSeconds(2)
//                                .forward(4)
//                                .strafeRight(26)
                                //far
//                                .splineTo(new Vector2d(-36, 32), Math.toRadians(-150))
//                                .lineToLinearHeading(new Pose2d(-34, 32, Math.toRadians(-120)))
//                                .lineToLinearHeading(new Pose2d(-34, 11, Math.toRadians(0)))
//                                .setTangent(0)
//                                .splineTo(new Vector2d(22, 11), Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(59, 25, Math.toRadians(0)), Math.toRadians(0))
                                //close side cycle
                                .setReversed(true)
                                .setTangent(Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(23, 42, Math.toRadians(-90)), Math.toRadians(-90))
                                .waitSeconds(1)
                                .setTangent(Math.toRadians(15))
                                //.splineToConstantHeading(new Vector2d(11, 32), Math.toRadians(-80))
                                .splineToSplineHeading(new Pose2d(48, 30, Math.toRadians(0)), Math.toRadians(-30))
                                .waitSeconds(1)
                                .setTangent(Math.toRadians(135))
                                .splineToSplineHeading(new Pose2d(15, 60,Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-24, 60,Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(1)
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-24, 60,Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(15, 60,Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(48, 42,Math.toRadians(0)), Math.toRadians(-45))
                                .waitSeconds(1)
                                .setTangent(Math.toRadians(135))
                                .splineToSplineHeading(new Pose2d(15, 60,Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-24, 60,Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(1)
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-24, 60,Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(15, 60,Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(48, 42,Math.toRadians(0)), Math.toRadians(-45))
                                .waitSeconds(0.5)
                                //.back(4)

                                //far cycle
//                                .setTangent(Math.toRadians(-120))
//                                .setReversed(true)
//                                .splineToLinearHeading(new Pose2d(-50, 18, Math.toRadians(60)), Math.toRadians(-120))
//                                .waitSeconds(1)
//                                .setTangent(Math.toRadians(-135))
//                                .splineToLinearHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
//                                .waitSeconds(1)
//                                .setTangent(0)
//                                //.splineToSplineHeading(new Pose2d(10, 12, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(18, 12, Math.toRadians(-150)), Math.toRadians(0))
//                                //.splineToLinearHeading(new Pose2d(46, 31, Math.toRadians(0)), Math.toRadians(45))
//                                .waitSeconds(0.75)
//                                .splineToSplineHeading(new Pose2d(48, 42, Math.toRadians(180)), Math.toRadians(60))
//                                .waitSeconds(1)
//                                .setTangent(Math.toRadians(-120))
//                                .splineToSplineHeading(new Pose2d(15, 12,Math.toRadians(180)), Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
//                                .waitSeconds(1)
//                                .setTangent(0)
//                                .splineToSplineHeading(new Pose2d(20, 12,Math.toRadians(180)), Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(48, 34,Math.toRadians(180)), Math.toRadians(45))
//                                .waitSeconds(0.5)
//                                .setTangent(Math.toRadians(-135))
//                                .splineToSplineHeading(new Pose2d(15, 12,Math.toRadians(180)), Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(-58, 12, Math.toRadians(180)), Math.toRadians(180))
//                                .waitSeconds(1)
//                                .setTangent(0)
//                                .splineToSplineHeading(new Pose2d(20, 12,Math.toRadians(180)), Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(48, 34,Math.toRadians(180)), Math.toRadians(45))
//                                .waitSeconds(0.5)
//                                .back(4)
                                .build()
                );

//        RoadRunnerBotEntity closePartner = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(40, 40, Math.toRadians(120), Math.toRadians(120), 15)
//                .setDimensions(13.5, 14) // Set size of bot
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(16.5, 63.5, Math.toRadians(90)))
//                                .setReversed(true)
//                                .setTangent(Math.toRadians(-90))
//                                .splineToSplineHeading(new Pose2d(24, 32, Math.toRadians(90)), Math.toRadians(-90))
//                                .waitSeconds(2)
//                                .setTangent(Math.toRadians(0))
//                                //.splineToConstantHeading(new Vector2d(11, 32), Math.toRadians(-80))
//                                .splineToSplineHeading(new Pose2d(48, 30, Math.toRadians(180)), Math.toRadians(0))
//                                .waitSeconds(2)
//                                .forward(4)
//                                .strafeRight(26)
//                                .build()
//                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
//                .addEntity(closePartner)
                .start();
    }
}