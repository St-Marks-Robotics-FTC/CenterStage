package org.firstinspires.ftc.teamcode.Testing.General;

import com.acmerobotics.dashboard.config.Config;
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
@Config
//@Disabled
@Autonomous(group = "drive")
public class PathTest extends LinearOpMode {
    public static double DISTANCE = -2;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-41, 60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose) // left
                .strafeLeft(DISTANCE)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        

        drive.followTrajectorySequence(left);

    }
}
