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
                        drive.trajectorySequenceBuilder(new Pose2d(-40, 63, Math.toRadians(90))) // Blue Far



//                                .splineToSplineHeading(new Pose2d(-34, -32, Math.toRadians(180)), Math.toRadians(35))
//                                // place purple from back


                        //drive.trajectorySequenceBuilder(new Pose2d(17, 63.5, Math.toRadians(-90))) // Blue Close
                                //close
//                                .setReversed(true)
//                                .setTangent(Math.toRadians(-90))
//                                .splineToSplineHeading(new Pose2d(23, 42, Math.toRadians(90)), Math.toRadians(-90))
//                                .setTangent(Math.toRadians(15))
//                                //.splineToConstantHeading(new Vector2d(11, 32), Math.toRadians(-80))
//                                .splineToSplineHeading(new Pose2d(48, 30, Math.toRadians(180)), Math.toRadians(-30))
                                //far
//                                .setTangent(Math.toRadians(-110))
//                                .splineToSplineHeading(new Pose2d(-42, 40, Math.toRadians(60)), Math.toRadians(-110))
//                                .setTangent(Math.toRadians(45))
//                                .splineToLinearHeading(new Pose2d(-26, 12, Math.toRadians(180)), Math.toRadians(0))
//                                .setTangent(0)
//                                .splineToConstantHeading(new Vector2d(25, 12) , Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(48, 30), Math.toRadians(0))
                                //close side cycle
//                                .setReversed(true)
//                                .setTangent(Math.toRadians(-90))
//                                .splineToSplineHeading(new Pose2d(23, 42, Math.toRadians(90)), Math.toRadians(-90))
//                                .waitSeconds(1)
//                                .setTangent(Math.toRadians(15))
//                                //.splineToConstantHeading(new Vector2d(11, 32), Math.toRadians(-80))
//                                .splineToSplineHeading(new Pose2d(48, 30, Math.toRadians(180)), Math.toRadians(-30))
//                                .waitSeconds(1)
//                                .setTangent(Math.toRadians(-135))
//                                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(-61, 12), Math.toRadians(180))
//                                .waitSeconds(2)
//                                .setTangent(Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(48, 34), Math.toRadians(45))
//                                .waitSeconds(1)
//                                .setTangent(Math.toRadians(-135))
//                                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(-61, 12), Math.toRadians(180))
//                                .waitSeconds(2)
//                                .setTangent(Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(48, 34), Math.toRadians(45))
//                                .waitSeconds(1)

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
                                .splineToConstantHeading(new Vector2d(42, 31), Math.toRadians(60))
                                .waitSeconds(1)
                                .setTangent(Math.toRadians(-135))
                                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-61, 12), Math.toRadians(180))
                                .waitSeconds(2)
                                .setTangent(0)
                                .splineToConstantHeading(new Vector2d(20, 12) , Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(42, 31), Math.toRadians(60))
                                .waitSeconds(1)
                                .setTangent(Math.toRadians(-135))
                                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-61, 12), Math.toRadians(180))
                                .waitSeconds(2)
                                .setTangent(0)
                                .splineToConstantHeading(new Vector2d(20, 12) , Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(42, 31), Math.toRadians(60))
                                .waitSeconds(1)

//                                .setReversed(true)
//                                .setTangent(Math.toRadians(-110))
//                                .splineToSplineHeading(new Pose2d(-42,24, Math.toRadians(180)), Math.toRadians(-30))
//                                .waitSeconds(1)
//                                .setTangent(Math.toRadians(180))
                                //.splineToSplineHeading(new Pose2d(-32, 24, Math.toRadians(180)), Math.toRadians(-90))
//                                .splineToConstantHeading(new Vector2d(-42, 12), Math.toRadians(0))
//                                .waitSeconds(1)
//                                .setTangent(0)
//                                .splineToSplineHeading(new Pose2d(20, 12, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(48, 36, Math.toRadians(180)), Math.toRadians(60))
//





                                // Blue close
//                                .setReversed(true)
////                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
////                                    robot.intake.tiltUp();
////                                    robot.outtake.openBothClaw();
////                                })
////                                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {robot.outtake.v4barTransfer();})
////                                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {robot.outtake.closeBothClaw();})
////                                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
////                                    robot.intake.tiltDown();
////                                    robot.outtake.v4barScore();
////                                })
//                                .splineToSplineHeading(new Pose2d(23, 34, Math.toRadians(90)), Math.toRadians(-90))
////                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.special.releasePixel();})
//                                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.transfer();})
//                                .waitSeconds(1)
//
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



//                                .setReversed(true)
//                                .setTangent(Math.toRadians(80))
////                                .UNSTABLE_addTemporalMarkerOffset(0.75, ()->{robot.jankOuttake.armDown();})
////                                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.jankOuttake.closeBoth();})
//                                .splineToSplineHeading(new Pose2d(-32,-36, Math.toRadians(-160)), Math.toRadians(30))
////                                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.jankOuttake.openLeft();})
////                                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{robot.jankOuttake.scoreArm();})
//                                .waitSeconds(1.5)
//                                .setTangent(Math.toRadians(180))
//                                //.splineToSplineHeading(new Pose2d(-32, 24, Math.toRadians(180)), Math.toRadians(-90))
//                                .splineToConstantHeading(new Vector2d(-32, -12), Math.toRadians(0))
//                                .waitSeconds(1)
//                                .setTangent(0)
//                                .splineToSplineHeading(new Pose2d(20, -12, Math.toRadians(180)), Math.toRadians(0))
//                                .splineToSplineHeading(new Pose2d(48, -30, Math.toRadians(180)), Math.toRadians(-60))
////                                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{robot.jankOuttake.openRight();})
//                                .forward(4)
//                                .strafeLeft(10)












                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}