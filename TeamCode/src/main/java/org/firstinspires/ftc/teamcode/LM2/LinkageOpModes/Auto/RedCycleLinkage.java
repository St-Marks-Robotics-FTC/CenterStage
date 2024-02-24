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
import org.firstinspires.ftc.teamcode.Vision.Prop.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled

@Config
@Autonomous
public class RedCycleLinkage extends LinearOpMode {

    FtcDashboard dashboard;

    public static String loc = "left";
    MecanumDrive drive;
    Linkageclass robot;

    private VisionPortal portal;
    private RedPropThreshold redPropThreshold;


    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        redPropThreshold = new RedPropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(redPropThreshold)
                .build();



        drive = new MecanumDrive(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(12, -60, Math.toRadians(90)));
        robot = new Linkageclass(hardwareMap);

        Pose2d startPose = new Pose2d(15, -60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(11, -29, Math.toRadians(160)), Math.toRadians(130))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.openRight();}) // score purple Preload
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})


                .setTangent(Math.toRadians(-15))
                .splineToSplineHeading(new Pose2d(52, -30, Math.toRadians(0)), Math.toRadians(-15))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score yellow Preload
                .waitSeconds(1)
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.setArm(70);})
                .setTangent(Math.toRadians(-135))
                .splineToSplineHeading(new Pose2d(15, -60,Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-24, -60,Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-61, -36, Math.toRadians(180)), Math.toRadians(180))//stack
                //close claw
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(1)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-24, -60,Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(15, -60,Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})
                .splineToSplineHeading(new Pose2d(51.5, -42,Math.toRadians(0)), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score pixels
                .waitSeconds(1)

                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(15, -31, Math.toRadians(90)), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.openRight();}) // score purple Preload
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})


                .setTangent(Math.toRadians(20))
                .splineToSplineHeading(new Pose2d(51.5, -36, Math.toRadians(0)), Math.toRadians(-30))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score yellow Preload
                .waitSeconds(1)
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.setArm(70);})
                .setTangent(Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(15, -60,Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-24, -60,Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-61, -36, Math.toRadians(180)), Math.toRadians(180))//stack
                //close claw
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(1)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-24, -60,Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(15, -60,Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})
                .splineToSplineHeading(new Pose2d(51.5, -42,Math.toRadians(0)), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score pixels
                .waitSeconds(1)

                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(23, -42, Math.toRadians(90)), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.openRight();}) // score purple Preload
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})
                .setTangent(Math.toRadians(15))
                //.splineToConstantHeading(new Vector2d(11, 32), Math.toRadians(-80))
                .splineToSplineHeading(new Pose2d(51.5, -42, Math.toRadians(0)), Math.toRadians(-30))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score yellow Preload
                .waitSeconds(1)
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.setArm(70);})
                .setTangent(Math.toRadians(135))
                .splineToSplineHeading(new Pose2d(15, -60,Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-24, -60,Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-61, -36, Math.toRadians(180)), Math.toRadians(180))//stack
                //close claw
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(1)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-24, -60,Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(15, -60,Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})
                .splineToSplineHeading(new Pose2d(51.5, -38,Math.toRadians(0)), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score pixels
                .waitSeconds(1)
                .build();



        robot.closeClaw();
        robot.holdDrone();
        while (opModeInInit()) {
            loc = redPropThreshold.getPropPosition();
            telemetry.addData("Prop Position", redPropThreshold.getPropPosition());
            telemetry.addData("Avg Left Value", redPropThreshold.getAvergageLeft());
            telemetry.addData("Avg Right Value", redPropThreshold.getAvergageRight());
            telemetry.update();
        }

        waitForStart();
        robot.setArm(125);
        robot.retractLinkage();
        robot.wristPickup();
        switch (loc) {
            case "none":
                drive.followTrajectorySequence(right); // flipped
                break;
            case "right":
                drive.followTrajectorySequence(left); // flipped
                break;
            case "left":
                drive.followTrajectorySequence(middle);
                break;
        }
    }
}
