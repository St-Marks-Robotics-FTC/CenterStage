package org.firstinspires.ftc.teamcode.LM2.LinkageOpModes.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

//@Disabled

@Disabled
@Config
@Autonomous
public class BlueCycleLinkage extends LinearOpMode {

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

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(11, 29, Math.toRadians(-160)), Math.toRadians(-130))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.openRight();}) // score purple Preload
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})


                .setTangent(Math.toRadians(20))
                .splineToSplineHeading(new Pose2d(52, 24, Math.toRadians(0)), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score yellow Preload
                .waitSeconds(1.5)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.setArm(80);})
                .setTangent(Math.toRadians(-180))
                .splineToSplineHeading(new Pose2d(11.5, 8, Math.toRadians(-180)), Math.toRadians(-180)) // going to stack
                .splineToSplineHeading(new Pose2d(-11.5, 8, Math.toRadians(-180)), Math.toRadians(-180))

                .splineToSplineHeading(new Pose2d(-60, 5, Math.toRadians(-180)), Math.toRadians(-180)) // stack
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(1.5)


                // stack to board
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-20, 9.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(11.5, 9.5, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(390);})
                .splineToConstantHeading(new Vector2d(52.5, 26), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.scoreRight();})
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d( 53, 32, Math.toRadians(0)))
                .waitSeconds(1)

                .back(4)

                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(15, 31, Math.toRadians(-90)), Math.toRadians(-90))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.openRight();}) // score purple Preload
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})


                .setTangent(Math.toRadians(20))
                .splineToSplineHeading(new Pose2d(51.5, 32, Math.toRadians(0)), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score yellow Preload
                .waitSeconds(1.5)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.setArm(80);})
                .setTangent(Math.toRadians(-160))
                .splineToSplineHeading(new Pose2d(11.5, 8, Math.toRadians(-180)), Math.toRadians(-180)) // going to stack
                .splineToSplineHeading(new Pose2d(-11.5, 8, Math.toRadians(-180)), Math.toRadians(-180))

                .splineToSplineHeading(new Pose2d(-60.5, 10, Math.toRadians(-180)), Math.toRadians(-180)) // stack
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(1.5)


                // stack to board
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-20, 9.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(11.5, 7, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(360);})
                .splineToConstantHeading(new Vector2d(52.5, 26), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d( 51, 34, Math.toRadians(0)))
                .waitSeconds(1)
                .back(4)

                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(15, 35, Math.toRadians(-45)), Math.toRadians(-90))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.openRight();}) // score purple Preload
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})


                .setTangent(Math.toRadians(20))
                .splineToSplineHeading(new Pose2d(51.5, 37, Math.toRadians(0)), Math.toRadians(0))


                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score yellow Preload
                .waitSeconds(1.5)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.setArm(70);})
                .setTangent(Math.toRadians(-140))
                .splineToSplineHeading(new Pose2d(11.5, 8, Math.toRadians(-180)), Math.toRadians(-180)) // going to stack
                .splineToSplineHeading(new Pose2d(-11.5, 8, Math.toRadians(-180)), Math.toRadians(-180))

                .splineToSplineHeading(new Pose2d(-62, 7, Math.toRadians(-180)), Math.toRadians(-180)) // stack
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(1.5)


                // stack to board
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-20, 9, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(11.5, 9, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})
                .splineToConstantHeading(new Vector2d(51.5, 24), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d( 50, 32, Math.toRadians(0)))
                .waitSeconds(1)

                .back(4)

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
        robot.setArm(35);
        robot.retractLinkage();
        robot.wristPickup();
        switch (loc) {
            case "none":
                drive.followTrajectorySequence(right);
                break;
            case "right":
                drive.followTrajectorySequence(middle);
                break;
            case "left":
                drive.followTrajectorySequence(left);
                break;
        }
    }
}