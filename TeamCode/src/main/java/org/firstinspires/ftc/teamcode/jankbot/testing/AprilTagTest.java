package org.firstinspires.ftc.teamcode.jankbot.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Vision.AprilTag.AprilTagRelocalize;
import org.firstinspires.ftc.teamcode.jankbot.Jankbot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class AprilTagTest extends LinearOpMode {

    private Jankbot robot;
    private AprilTagRelocalize relocalize;
    private int tag = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Jankbot(hardwareMap);
        relocalize = new AprilTagRelocalize(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        Pose2d relocalizePose = new Pose2d();

        waitForStart();
        relocalizePose = relocalize.getTagPos(tag);
        TrajectorySequence traj1 = robot.drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(relocalizePose)
                .build();

        robot.drive.followTrajectorySequence(traj1);
    }
}
