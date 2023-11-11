package org.firstinspires.ftc.teamcode.armbot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Vision.PropLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;

@Autonomous
public class Auto extends LinearOpMode {

    FtcDashboard dashboard;
    PropLocalizer propLocalizer;
    private static int loc = 0;
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

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(-90)))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(12, -44, Math.toRadians(-90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(8, -30, Math.toRadians(-30)), Math.toRadians(120))
                .build();
        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(-90)))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(12, -30, Math.toRadians(-90)), Math.toRadians(90))
                .build();
        TrajectorySequence traj13 = drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(-90)))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(18, -30, Math.toRadians(-120)), Math.toRadians(90))
                .build();
        TrajectorySequence traj21 = drive.trajectorySequenceBuilder(traj11.end())
                .setTangent(Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(48, -32, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj22 = drive.trajectorySequenceBuilder(traj12.end())
                .setTangent(Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(48, -36, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj23 = drive.trajectorySequenceBuilder(traj13.end())
                .setTangent(Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(48, -40, Math.toRadians(0)), Math.toRadians(0))
                .build();

//        TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .strafeLeft(12)
//                .build();

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
        robot.setArm(1000);

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

//        drive.followTrajectorySequence(park);

    }
}
