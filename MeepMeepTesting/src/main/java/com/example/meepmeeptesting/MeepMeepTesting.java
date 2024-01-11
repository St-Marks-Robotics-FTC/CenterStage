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
                .setConstraints(60, 50, Math.toRadians(120), Math.toRadians(120), 15)
                .setDimensions(13.5, 14) // Set size of bot
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-41, 64, Math.toRadians(90))) // Blue Far
                        //drive.trajectorySequenceBuilder(new Pose2d(17, 64, Math.toRadians(90))) // Blue Close
                                //close
//                                .setTangent(Math.toRadians(-90))
//                                .splineToSplineHeading(new Pose2d(19, 37, Math.toRadians(125)), Math.toRadians(-55))
//                                .setTangent(Math.toRadians(135))
//                                .splineToConstantHeading(new Vector2d(11, 32), Math.toRadians(-80))
//                                .splineToSplineHeading(new Pose2d(48, 30, Math.toRadians(180)), Math.toRadians(30))
                                //far
//                                .setTangent(Math.toRadians(-110))
//                                .splineToSplineHeading(new Pose2d(-42, 40, Math.toRadians(60)), Math.toRadians(-110))
//                                .setTangent(Math.toRadians(45))
//                                .splineToLinearHeading(new Pose2d(-26, 12, Math.toRadians(180)), Math.toRadians(0))
//                                .setTangent(0)
//                                .splineToConstantHeading(new Vector2d(25, 12) , Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(48, 30), Math.toRadians(0))
                                //close side cycle
//                                .setTangent(Math.toRadians(-90))
//                                .splineToSplineHeading(new Pose2d(19, 37, Math.toRadians(125)), Math.toRadians(-55))
//                                .waitSeconds(1)
//                                .setTangent(Math.toRadians(135))
//                                .splineToConstantHeading(new Vector2d(11, 32), Math.toRadians(-80))
//                                .splineToSplineHeading(new Pose2d(48, 30, Math.toRadians(180)), Math.toRadians(30))
//                                .waitSeconds(1)
//                                .setTangent(Math.toRadians(-135))
//                                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(-61, 12), Math.toRadians(180))
//                                .waitSeconds(2)
//                                .setTangent(Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(48, 34), Math.toRadians(45))
//                                .waitSeconds(1.5)
//                                .setTangent(Math.toRadians(-135))
//                                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(-61, 12), Math.toRadians(180))
//                                .waitSeconds(2)
//                                .setTangent(Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(48, 34), Math.toRadians(45))
//                                .waitSeconds(1.5)
//                                .setTangent(Math.toRadians(-135))
//                                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(-61, 12), Math.toRadians(180))
//                                .waitSeconds(2)
//                                .setTangent(Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(48, 34), Math.toRadians(45))
//                                .waitSeconds(1.5)
                                //far cycle
                                //.setTangent(Math.toRadians(-120))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-50, 20, Math.toRadians(-135)), Math.toRadians(-110))
                                .waitSeconds(2)
                                .setTangent(Math.toRadians(-135))
                                .splineToLinearHeading(new Pose2d(-61, 12, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(2)
                                .setTangent(0)
                                .splineToConstantHeading(new Vector2d(20, 12) , Math.toRadians(0))
                                //.splineToLinearHeading(new Pose2d(46, 31, Math.toRadians(0)), Math.toRadians(45))
                                .splineToConstantHeading(new Vector2d(46, 31), Math.toRadians(45))
                                .waitSeconds(1)
                                .setTangent(Math.toRadians(-135))
                                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-61, 12), Math.toRadians(180))
                                .waitSeconds(2)
                                .setTangent(0)
                                .splineToConstantHeading(new Vector2d(20, 12) , Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(46, 31), Math.toRadians(45))
                                .waitSeconds(1)
                                .setTangent(Math.toRadians(-135))
                                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-61, 12), Math.toRadians(180))
                                .waitSeconds(2)
                                .setTangent(0)
                                .splineToConstantHeading(new Vector2d(20, 12) , Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(46, 31), Math.toRadians(45))
                                .waitSeconds(1)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}