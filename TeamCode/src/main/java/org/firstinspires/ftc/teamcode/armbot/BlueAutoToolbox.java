package org.firstinspires.ftc.teamcode.armbot;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.BluePropThreshold;
import org.firstinspires.ftc.teamcode.Vision.RedPropThreshold;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous
public class BlueAutoToolbox extends LinearOpMode {

    FtcDashboard dashboard;

    public static String loc = "left";
    MecanumDrive drive;
    Arm robot;

    private VisionPortal portal;
    private BluePropThreshold bluePropThreshold;


    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bluePropThreshold = new BluePropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(bluePropThreshold)
                .build();



        drive = new MecanumDrive(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(12, -60, Math.toRadians(90)));
        robot = new Arm(hardwareMap);

        Pose2d startPose = new Pose2d(15, 60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(startPose) // right side
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(12, 44, Math.toRadians(90)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(6, 30, Math.toRadians(20)), Math.toRadians(-120))
                .setReversed(false)
                .build();
        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(startPose) // middle
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(15, 29, Math.toRadians(90)), Math.toRadians(-90))
                .setReversed(false)
                .build();
        TrajectorySequence traj13 = drive.trajectorySequenceBuilder(startPose) // left
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(21, 30, Math.toRadians(90)), Math.toRadians(-90))
                .setReversed(false)
                .build();
        TrajectorySequence traj21 = drive.trajectorySequenceBuilder(traj11.end())
                .setTangent(Math.toRadians(20))
                .splineToSplineHeading(new Pose2d(50, 27, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj22 = drive.trajectorySequenceBuilder(traj12.end())
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(50, 32, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj23 = drive.trajectorySequenceBuilder(traj13.end())
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(50, 40, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(traj21.end())
//                .strafeRight(18)
                .strafeLeft(28)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(traj22.end())
//                .strafeRight(22)
                .strafeLeft(22)

                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(traj23.end())
//                .strafeRight(28)
                .strafeLeft(18)
                .build();

        robot.closeClaw();
        while (opModeInInit()) {
            loc = bluePropThreshold.getPropPosition();
            telemetry.addData("Prop Position", bluePropThreshold.getPropPosition());
            telemetry.addData("Avg Left Value", bluePropThreshold.getAvergageLeft());
            telemetry.addData("Avg Right Value", bluePropThreshold.getAvergageRight());
            telemetry.update();
        }

        waitForStart();
        robot.setArm(-700);
        sleep(1000);
        switch (loc) {
            case "right":
                drive.followTrajectorySequence(traj11);
                break;
            case "center":
                drive.followTrajectorySequence(traj12);
                break;
            case "left":
                drive.followTrajectorySequence(traj13);
                break;
        }

        //robot.openAutoClaw();
        sleep(3000);
        robot.setArm(-1650); // 390

        //outtake
        switch (loc) {
            case "right":
                drive.followTrajectorySequence(traj21);
                break;
            case "center":
                drive.followTrajectorySequence(traj22);
                break;
            case "left":
                drive.followTrajectorySequence(traj23);
                break;
        }

        robot.openClaw();

        sleep(1000);
        robot.setArm(-800); // 700

        switch (loc) {
            case "right":
                drive.followTrajectorySequence(park1);
                break;
            case "center":
                drive.followTrajectorySequence(park2);
                break;
            case "left":
                drive.followTrajectorySequence(park3);
                break;
        }
        robot.setArm(-1600); // 700
        sleep(1000);

    }
}
