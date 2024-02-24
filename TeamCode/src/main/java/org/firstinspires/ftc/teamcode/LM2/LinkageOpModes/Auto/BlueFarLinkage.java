package org.firstinspires.ftc.teamcode.LM2.LinkageOpModes.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LM2.LM2class;
import org.firstinspires.ftc.teamcode.LM2.Linkageclass;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.Prop.BlueFarPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous
public class BlueFarLinkage extends LinearOpMode {

    FtcDashboard dashboard;

    public static String loc = "left";
    MecanumDrive drive;
    Linkageclass robot;

    private VisionPortal portal;
    private BlueFarPropThreshold blueFarPropThreshold;


    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        blueFarPropThreshold = new BlueFarPropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(blueFarPropThreshold)
                .build();



        drive = new MecanumDrive(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(12, -60, Math.toRadians(90)));
        robot = new Linkageclass(hardwareMap);

        Pose2d startPose = new Pose2d(-38, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(startPose) // left
                .splineToSplineHeading(new Pose2d(-31, 34, Math.toRadians(-30)), Math.toRadians(0))
                .build();
        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(startPose) // middle
                .splineToSplineHeading(new Pose2d(-40, 26.5, Math.toRadians(-10)), Math.toRadians(-10))
                .build();
        TrajectorySequence traj13 = drive.trajectorySequenceBuilder(startPose) // right
                .splineTo(new Vector2d(-38, 32), Math.toRadians(-130))
                .build();
        TrajectorySequence traj21 = drive.trajectorySequenceBuilder(traj11.end())
                //.splineTo(new Vector2d(-41, -32), Math.toRadians(150))
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-38, 34, Math.toRadians(0)), Math.toRadians(-180))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-31, 9.5), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(22, 9.5, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(390);})
                .splineToSplineHeading(new Pose2d(53, 34, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj22 = drive.trajectorySequenceBuilder(traj12.end())
                .splineToConstantHeading(new Vector2d(-48, 22), Math.toRadians(-180))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-31, 8), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(22, 8, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(390);})
                .splineToSplineHeading(new Pose2d(54, 27.5, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj23 = drive.trajectorySequenceBuilder(traj13.end())
                .lineToLinearHeading(new Pose2d(-34, 32, Math.toRadians(-120)))
                .lineToLinearHeading(new Pose2d(-34, 11, Math.toRadians(0)))
                .setTangent(0)
                .splineTo(new Vector2d(22, 11), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(390);})
                .splineToSplineHeading(new Pose2d(53, 21.2, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(traj21.end())
                .back(8)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(traj22.end())
                .back(8)
                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(traj23.end())
                .back(8)
                .build();

        robot.holdDrone();
        robot.closeClaw();
        while (opModeInInit()) {
            loc = blueFarPropThreshold.getPropPosition();
            telemetry.addData("Prop Position", blueFarPropThreshold.getPropPosition());
            telemetry.addData("Avg Left Value", blueFarPropThreshold.getAvergageLeft());
            telemetry.addData("Avg Right Value", blueFarPropThreshold.getAvergageRight());
            telemetry.update();
        }

        waitForStart();
        robot.setArm(20);
        robot.retractLinkage();
        robot.wristPickup();

        sleep(10000);
        switch (loc) {
            case "none":
                drive.followTrajectorySequence(traj11);
                break;
            case "left":
                drive.followTrajectorySequence(traj12);
                break;
            case "right":
                drive.followTrajectorySequence(traj13);
                break;
        }

        robot.openRight();
        sleep(3000);
        robot.setArm(95); // 390

        //outtake
        switch (loc) {
            case "none":
                drive.followTrajectorySequence(traj21);
                break;
            case "left":
                drive.followTrajectorySequence(traj22);
                break;
            case "right":
                drive.followTrajectorySequence(traj23);
                break;
        }

        //robot.openClaw();
        robot.openLeft();
        sleep(500);
        switch (loc) {
            case "none":
                drive.followTrajectorySequence(park1);
                break;
            case "left":
                drive.followTrajectorySequence(park2);
                break;
            case "right":
                drive.followTrajectorySequence(park3);
                break;
        }
        robot.setLinkage(0.25);
        robot.setArm(5);
        sleep(2000);
    }
}
