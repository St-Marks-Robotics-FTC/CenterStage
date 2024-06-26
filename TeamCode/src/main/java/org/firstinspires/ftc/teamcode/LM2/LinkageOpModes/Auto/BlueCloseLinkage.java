package org.firstinspires.ftc.teamcode.LM2.LinkageOpModes.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LM2.LM2class;
import org.firstinspires.ftc.teamcode.LM2.Linkageclass;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.Prop.BluePropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Config
@Autonomous
public class BlueCloseLinkage extends LinearOpMode {

    FtcDashboard dashboard;

    public static String loc = "left";
    MecanumDrive drive;
    Linkageclass robot;

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
        robot = new Linkageclass(hardwareMap);

        Pose2d startPose = new Pose2d(15, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(startPose) // right side
                .splineToSplineHeading(new Pose2d(11, 29, Math.toRadians(-160)), Math.toRadians(-130))
                .build();
        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(startPose) // middle
                .splineToSplineHeading(new Pose2d(15, 31, Math.toRadians(-90)), Math.toRadians(-90))
                .build();
        TrajectorySequence traj13 = drive.trajectorySequenceBuilder(startPose) // left
                .splineToSplineHeading(new Pose2d(14, 34, Math.toRadians(-45)), Math.toRadians(-90))
                .build();
        TrajectorySequence traj21 = drive.trajectorySequenceBuilder(traj11.end())
                .setTangent(Math.toRadians(20))
                .splineToSplineHeading(new Pose2d(51, 24, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj22 = drive.trajectorySequenceBuilder(traj12.end())
                .setTangent(Math.toRadians(20))
                .splineToSplineHeading(new Pose2d(51, 32, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj23 = drive.trajectorySequenceBuilder(traj13.end())
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(51, 37, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(traj21.end())
//                .strafeRight(18)
                .back(5)
                .strafeLeft(28)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(traj22.end())
//                .strafeRight(22)
                .back(5)
                .strafeLeft(22)

                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(traj23.end())
//                .strafeRight(28)
                .back(5)
                .strafeLeft(18)
                .build();

        robot.holdDrone();
        robot.closeClaw();
        while (opModeInInit()) {
            loc = bluePropThreshold.getPropPosition();
            telemetry.addData("Prop Position", bluePropThreshold.getPropPosition());
            telemetry.addData("Avg Left Value", bluePropThreshold.getAvergageLeft());
            telemetry.addData("Avg Right Value", bluePropThreshold.getAvergageRight());
            telemetry.update();
        }

        waitForStart();
        robot.setArm(55);
        robot.retractLinkage();
        robot.wristPickup();
        //robot.setArm(-700);
//        sleep(1000);
        switch (loc) {
            case "none":
                drive.followTrajectorySequence(traj11);
                break;
            case "right":
                drive.followTrajectorySequence(traj12);
                break;
            case "left":
                drive.followTrajectorySequence(traj13);
                break;
        }

        robot.openRight();
        sleep(150);
        robot.setArm(370); // 390

        //outtake
        switch (loc) {
            case "none":
                drive.followTrajectorySequence(traj21);
                break;
            case "right":
                drive.followTrajectorySequence(traj22);
                break;
            case "left":
                drive.followTrajectorySequence(traj23);
                break;
        }

        robot.openLeft();

        sleep(150);

        switch (loc) {
            case "none":
                drive.followTrajectorySequence(park1);
                break;
            case "right":
                drive.followTrajectorySequence(park2);
                break;
            case "left":
                drive.followTrajectorySequence(park3);
                break;
        }
        robot.setLinkage(0.25);
        robot.setArm(5);
        sleep(2000);

    }
}
