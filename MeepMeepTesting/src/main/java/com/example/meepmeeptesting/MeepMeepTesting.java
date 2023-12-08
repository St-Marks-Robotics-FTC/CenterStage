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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, -60, Math.toRadians(90))) // Red
//                        drive.trajectorySequenceBuilder(new Pose2d(15, 60, Math.toRadians(90))) // Blue
//                                drive.trajectorySequenceBuilder(new Pose2d(-38, -60, Math.toRadians(90))) // Red Far

                  // BLUE SIDE
//                                // right
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(12, 44, Math.toRadians(90)), Math.toRadians(-90))
//                                .splineToSplineHeading(new Pose2d(6, 30, Math.toRadians(20)), Math.toRadians(-120))
//                                .setReversed(false)
//                                .setTangent(Math.toRadians(20))
//                                .splineToSplineHeading(new Pose2d(50, 27, Math.toRadians(0)), Math.toRadians(0))

                                // middle
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(15, 29, Math.toRadians(90)), Math.toRadians(-90))
//                                .setReversed(false)
//                                .setTangent(Math.toRadians(90))
//                                .splineToSplineHeading(new Pose2d(50, 34, Math.toRadians(0)), Math.toRadians(0))

                                // left
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(23, 30, Math.toRadians(90)), Math.toRadians(-90))
//                                .setReversed(false)
//                                .setTangent(Math.toRadians(90))
//                                .splineToSplineHeading(new Pose2d(50, 40, Math.toRadians(0)), Math.toRadians(0))



                    // RED FAR SIDE
                                // right
                                        //.setReversed(true)
                                        .splineToSplineHeading(new Pose2d(-31, -34, Math.toRadians(30)), Math.toRadians(0))
                                        //.setReversed(false)
                                        .splineToSplineHeading(new Pose2d(-38, -34, Math.toRadians(0)), Math.toRadians(180))
                                        .splineToConstantHeading(new Vector2d(-31, -11), Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(11, -11, Math.toRadians(0)), Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(52, -39, Math.toRadians(0)), Math.toRadians(0))
                                        // middle
                                        //.setReversed(true)
//                                        .splineToSplineHeading(new Pose2d(-40, -26, Math.toRadians(10)), Math.toRadians(10))
//                                        //.setReversed(false)
//                                        //.splineToSplineHeading(new Pose2d(-31, -11, Math.toRadians(0)), Math.toRadians(0))
//                                        .splineToConstantHeading(new Vector2d(-45, -26), Math.toRadians(180))
//                                        .setTangent(Math.toRadians(90))
//                                        .splineToConstantHeading(new Vector2d(-31, -11), Math.toRadians(0))
//                                        .splineToSplineHeading(new Pose2d(11, -11, Math.toRadians(0)), Math.toRadians(0))
//                                        .splineToSplineHeading(new Pose2d(50, -36, Math.toRadians(0)), Math.toRadians(0))

                                        // left
                                        //.setReversed(true)
//                                        .splineTo(new Vector2d(-41, -32), Math.toRadians(150))
//                                        //.setReversed(false)
//                                        .lineToLinearHeading(new Pose2d(-34, -32, Math.toRadians(120)))
//                                        .lineToLinearHeading(new Pose2d(-34, -11, Math.toRadians(90)))
//                                        .setTangent(0)
//                                        .splineTo(new Vector2d(11, -11), Math.toRadians(0))
//                                        .splineToSplineHeading(new Pose2d(50, -27, Math.toRadians(0)), Math.toRadians(0))





                  // RED SIDE
//                                // right
//                                .splineToSplineHeading(new Pose2d(15, -27, Math.toRadians(45)), Math.toRadians(90))
//                                .setTangent(Math.toRadians(-20))
//                                .splineToSplineHeading(new Pose2d(42, -42, Math.toRadians(0)), Math.toRadians(0))
//                                // middle
//                                .splineToSplineHeading(new Pose2d(15, -33, Math.toRadians(90)), Math.toRadians(90))
//                                .setTangent(Math.toRadians(-20))
//                                .splineToSplineHeading(new Pose2d(42, -34, Math.toRadians(0)), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(-33, -34), Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(-50, -34, Math.toRadians(180)), Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(-33, -34, Math.toRadians(0)), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(42, -34), Math.toRadians(0))

//                                // left
                                //.setReversed(true)
//                                .splineToSplineHeading(new Pose2d(10, -33, Math.toRadians(160)), Math.toRadians(130))
//                                .setTangent(Math.toRadians(-20))
//                                .splineToSplineHeading(new Pose2d(42, -27, Math.toRadians(0)), Math.toRadians(0))
//                                //.setReversed(false)
//                                .splineToSplineHeading(new Pose2d(50, -27, Math.toRadians(0)), Math.toRadians(0))



                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}