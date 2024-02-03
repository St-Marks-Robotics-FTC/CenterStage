package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Fallback.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Fallback.Roadrunner.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class PathTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-41, 60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose) // left
                .setReversed(true)
                .setTangent(Math.toRadians(-110))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.intake.tiltUp();})
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.outtake.openBothClaw();})
//                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {robot.outtake.v4barTransfer();})
//                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {robot.outtake.closeBothClaw();})
//                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {robot.intake.tiltDown();})
//                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{robot.outtake.v4barScore();})
                .splineToSplineHeading(new Pose2d(-30, 33, Math.toRadians(150)), Math.toRadians(-30))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.special.releasePixel();})
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.transfer();})
                .waitSeconds(1)
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.outtake.v4barScore();})
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-45, 12, Math.toRadians(180)), Math.toRadians(0))
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(12, 12) , Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, 47), Math.toRadians(0))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.outtake.openBothClaw();})
                .waitSeconds(1.5)
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{robot.outtake.v4barStow();})
                .waitSeconds(0.5)
                .forward(4)
                .strafeLeft(18)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        

        drive.followTrajectorySequence(left);

    }
}
