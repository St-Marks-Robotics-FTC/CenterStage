package org.firstinspires.ftc.teamcode.armbot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Vision.PropLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class Auto extends LinearOpMode {

    FtcDashboard dashboard;
    PropLocalizer propLocalizer;
    public static int loc = 0;
    MecanumDrive drive;
    Arm robot;

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = FtcDashboard.getInstance();
        propLocalizer = new PropLocalizer(telemetry, hardwareMap, gamepad1, false, dashboard);
        propLocalizer.initLocalizer();
        drive = new MecanumDrive(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(12, -60, Math.toRadians(90)));
        robot = new Arm(hardwareMap);

        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(12, -44, Math.toRadians(-90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(6, -30, Math.toRadians(-20)), Math.toRadians(120))
                .setReversed(false)
                .build();
        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(15, -29, Math.toRadians(-90)), Math.toRadians(90))
                .setReversed(false)
                .build();
        TrajectorySequence traj13 = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(18, -30, Math.toRadians(-120)), Math.toRadians(90))
                .setReversed(false)
                .build();
        TrajectorySequence traj21 = drive.trajectorySequenceBuilder(traj11.end())
                .setTangent(Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(50, -27, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj22 = drive.trajectorySequenceBuilder(traj12.end())
                .setTangent(Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(50, -34, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj23 = drive.trajectorySequenceBuilder(traj13.end())
                .setTangent(Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(50, -40, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(traj21.end())
                .strafeLeft(18)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(traj22.end())
                .strafeLeft(22)
                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(traj23.end())
                .strafeLeft(28)
                .build();

        robot.closeClaw();
        while (!opModeIsActive() && !isStopRequested()) {
            propLocalizer.initLoop();
            loc = propLocalizer.getLoc();
            telemetry.addData("loc: ", loc);
            telemetry.addData("exposure: ", propLocalizer.curExp);
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            idle();
        }
        if (isStopRequested()) {
            propLocalizer.terminator();
        }

        waitForStart();
        robot.setArm(-700);
        sleep(1000);
        switch (loc) {
            case 0:
                drive.followTrajectorySequence(traj11);
                break;
            case 1:
                drive.followTrajectorySequence(traj12);
                break;
            case 2:
                drive.followTrajectorySequence(traj13);
                break;
        }

        robot.openAutoClaw();
        sleep(3000);
        robot.setArm(-1650); // 390

        //outtake
        switch (loc) {
            case 0:
                drive.followTrajectorySequence(traj21);
                break;
            case 1:
                drive.followTrajectorySequence(traj22);
                break;
            case 2:
                drive.followTrajectorySequence(traj23);
                break;
        }

        robot.openClaw();

        sleep(1000);
        robot.setArm(-800); // 700

        switch (loc) {
            case 0:
                drive.followTrajectorySequence(park1);
                break;
            case 1:
                drive.followTrajectorySequence(park2);
                break;
            case 2:
                drive.followTrajectorySequence(park3);
                break;
        }
        robot.setArm(-1600); // 700
        sleep(1000);

    }
}
