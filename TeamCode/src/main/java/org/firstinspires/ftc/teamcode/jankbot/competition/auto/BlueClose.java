package org.firstinspires.ftc.teamcode.jankbot.competition.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.Prop.BluePropThreshold;
import org.firstinspires.ftc.teamcode.armbot.BozoClass;
import org.firstinspires.ftc.teamcode.jankbot.Jankbot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous (group = "Blue", preselectTeleOp = "JankTele")
public class BlueClose extends LinearOpMode {
    FtcDashboard dashboard;

    public static String loc = "left";
    //MecanumDrive drive;
    Jankbot robot;

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



        //drive = new MecanumDrive(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(12, -60, Math.toRadians(90)));
        robot = new Jankbot(hardwareMap);

        Pose2d startPose = new Pose2d(15, 60, Math.toRadians(-90));
        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence traj11 = robot.drive.trajectorySequenceBuilder(startPose) // right side
                .splineToSplineHeading(new Pose2d(11, 29, Math.toRadians(-160)), Math.toRadians(-130))
                .build();
        TrajectorySequence traj12 = robot.drive.trajectorySequenceBuilder(startPose) // middle
                .splineToSplineHeading(new Pose2d(15, 31, Math.toRadians(-90)), Math.toRadians(-90))
                .build();
        TrajectorySequence traj13 = robot.drive.trajectorySequenceBuilder(startPose) // left
                .splineToSplineHeading(new Pose2d(16, 34, Math.toRadians(-45)), Math.toRadians(-90))
                .build();
        TrajectorySequence traj21 = robot.drive.trajectorySequenceBuilder(traj11.end())
                .setTangent(Math.toRadians(20))
                .splineToSplineHeading(new Pose2d(52, 24, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj22 = robot.drive.trajectorySequenceBuilder(traj12.end())
                .setTangent(Math.toRadians(20))
                .splineToSplineHeading(new Pose2d(52, 32, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj23 = robot.drive.trajectorySequenceBuilder(traj13.end())
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(52, 39, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence park1 = robot.drive.trajectorySequenceBuilder(traj21.end())
//                .strafeRight(18)
                .back(4)
                .strafeLeft(28)
                .build();
        TrajectorySequence park2 = robot.drive.trajectorySequenceBuilder(traj22.end())
//                .strafeRight(22)
                .back(4)
                .strafeLeft(22)

                .build();
        TrajectorySequence park3 = robot.drive.trajectorySequenceBuilder(traj23.end())
//                .strafeRight(28)
                .back(4)
                .strafeLeft(18)
                .build();

        //robot.closeClaw();
        while (opModeInInit()) {
            loc = bluePropThreshold.getPropPosition();
            telemetry.addData("Prop Position", bluePropThreshold.getPropPosition());
            telemetry.addData("Avg Left Value", bluePropThreshold.getAvergageLeft());
            telemetry.addData("Avg Right Value", bluePropThreshold.getAvergageRight());
            telemetry.update();
        }

        waitForStart();
        //robot.setArm(-700);
        sleep(1000);
        switch (loc) {
            case "right":
                robot.drive.followTrajectorySequence(traj11);
                break;
            case "center":
                robot.drive.followTrajectorySequence(traj12);
                break;
            case "left":
                robot.drive.followTrajectorySequence(traj13);
                break;
        }

        //robot.openRight();
        sleep(3000);
        //robot.setArm(350); // 390

        //outtake
        switch (loc) {
            case "right":
                robot.drive.followTrajectorySequence(traj21);
                break;
            case "center":
                robot.drive.followTrajectorySequence(traj22);
                break;
            case "left":
                robot.drive.followTrajectorySequence(traj23);
                break;
        }

        //robot.openLeft();

        sleep(1000);

        switch (loc) {
            case "right":
                robot.drive.followTrajectorySequence(park1);
                break;
            case "center":
                robot.drive.followTrajectorySequence(park2);
                break;
            case "left":
                robot.drive.followTrajectorySequence(park3);
                break;
        }
        //robot.setArm(0);

    }
}
