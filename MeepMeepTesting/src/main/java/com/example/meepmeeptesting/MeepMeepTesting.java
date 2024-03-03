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
                .setConstraints(70, 60, Math.toRadians(145), Math.toRadians(120), 15)
                .setDimensions(13.5, 14) // Set size of bot
                .followTrajectorySequence(drive ->
                        //drive.trajectorySequenceBuilder(new Pose2d(-41, 63.5, Math.toRadians(-90))) //  Far
                        drive.trajectorySequenceBuilder(new Pose2d(16.5, 63.5, Math.toRadians(90))) // Close
                                //Close Door
                                .setTangent(Math.toRadians(-90))
//                                .splineToSplineHeading(new Pose2d(11, 29, Math.toRadians(-160)), Math.toRadians(-130))
//                                .splineToSplineHeading(new Pose2d(16, 33, Math.toRadians(-90)), Math.toRadians(-90))
                                .splineToSplineHeading(new Pose2d(22, 42, Math.toRadians(-90)), Math.toRadians(-90))
                                .waitSeconds(1)
                                .setTangent(Math.toRadians(20))
                                //.splineToSplineHeading(new Pose2d(48, 30, Math.toRadians(180)), Math.toRadians(15))
                                //.splineToSplineHeading(new Pose2d(48, 36, Math.toRadians(180)), Math.toRadians(15))
                                .splineToSplineHeading(new Pose2d(48, 42, Math.toRadians(180)), Math.toRadians(15))
                                .waitSeconds(1)
                                //cycle time
                                //.back(4)
                                .setTangent(Math.toRadians(-135))
                                .splineToSplineHeading(new Pose2d(15, 10,Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-24, 10,Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-61, 10, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(1)
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-24, 10,Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(15, 10,Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(48, 36,Math.toRadians(180)), Math.toRadians(45))
                                .waitSeconds(1)
                                .setTangent(Math.toRadians(-135))
                                .splineToSplineHeading(new Pose2d(15, 10,Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-24, 10,Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-61, 10, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(1)
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-24, 10,Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(15, 10,Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(48, 36,Math.toRadians(180)), Math.toRadians(45))
                                .waitSeconds(1)
                                .setTangent(Math.toRadians(-135))
                                .splineToSplineHeading(new Pose2d(15, 10,Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-24, 10,Math.toRadians(180)), Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-61, 22, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(1)
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-24, 10,Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(15, 10,Math.toRadians(180)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(48, 36,Math.toRadians(180)), Math.toRadians(45))
                                .waitSeconds(0.5)


                                //Close Truss
//                                .setTangent(Math.toRadians(-90))
////                                .splineToSplineHeading(new Pose2d(11, 29, Math.toRadians(-160)), Math.toRadians(-130))
////                                .splineToSplineHeading(new Pose2d(16, 33, Math.toRadians(-90)), Math.toRadians(-90))
//                                .splineToSplineHeading(new Pose2d(22, 42, Math.toRadians(-90)), Math.toRadians(-90))
//                                .waitSeconds(1)
//                                .setTangent(Math.toRadians(20))
//                                //.splineToSplineHeading(new Pose2d(48, 30, Math.toRadians(180)), Math.toRadians(15))
//                                //.splineToSplineHeading(new Pose2d(48, 36, Math.toRadians(180)), Math.toRadians(15))
//                                .splineToSplineHeading(new Pose2d(48, 42, Math.toRadians(180)), Math.toRadians(15))
//                                .waitSeconds(1)
                                //cycle time
                                //.back(4)
//                                .setTangent(Math.toRadians(135))
//                                .splineToSplineHeading(new Pose2d(15, 60,Math.toRadians(180)), Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(-20, 60,Math.toRadians(180)), Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(180)), Math.toRadians(180))
//                                .waitSeconds(1)
//                                .setTangent(Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(-20, 60,Math.toRadians(180)), Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(15, 60,Math.toRadians(180)), Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(48, 36,Math.toRadians(180)), Math.toRadians(-45))
//                                .waitSeconds(1)
//                                .setTangent(Math.toRadians(135))
//                                .splineToSplineHeading(new Pose2d(15, 60,Math.toRadians(180)), Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(-20, 60,Math.toRadians(180)), Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(-61, 36, Math.toRadians(180)), Math.toRadians(180))
//                                .waitSeconds(1)
//                                .setTangent(Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(-20, 60,Math.toRadians(180)), Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(15, 60,Math.toRadians(180)), Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(48, 36,Math.toRadians(180)), Math.toRadians(-45))
//                                .waitSeconds(0.5)
                                //.back(4)

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