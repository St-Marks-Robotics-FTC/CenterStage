package org.firstinspires.ftc.teamcode.jankbot.competition.auto;

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
import org.firstinspires.ftc.teamcode.Vision.Prop.BluePropThreshold;
import org.firstinspires.ftc.teamcode.jankbot.Jankbot;
import org.firstinspires.ftc.teamcode.jankbot.competition.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Disabled
@Autonomous (group = "Blue", preselectTeleOp = "JankTele")
public class BlueClose extends LinearOpMode {
    FtcDashboard dashboard;

    public static String loc = "left";
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


        robot = new Jankbot(hardwareMap);

        Pose2d startPose = new Pose2d(17, 63.5, Math.toRadians(90));
        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startPose) // right side
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.tiltUp();
                    robot.outtake.openBothClaws();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {robot.outtake.v4barTransfer();})
                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {robot.outtake.closeBothClaws();})
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
                    robot.intake.tiltDown();
                    robot.outtake.v4barScore();
                })
                .splineToSplineHeading(new Pose2d(5, 36, Math.toRadians(30)), Math.toRadians(-130))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.special.releasePixel();})
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.transfer();})
                .waitSeconds(1) // Score Purple



                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.outtake.v4barScore();})
                .splineToSplineHeading(new Pose2d(50, 42, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.outtake.openBothClaws();})
                .waitSeconds(1.5)

                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{robot.outtake.v4barStow();})
                .waitSeconds(0.5)
                .forward(3)
                .lineToLinearHeading(new Pose2d(45, 12, Math.toRadians(-155)))
                .build();
        TrajectorySequence middle = robot.drive.trajectorySequenceBuilder(startPose) // middle
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.tiltUp();
                    robot.outtake.openBothClaws();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {robot.outtake.v4barTransfer();})
                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {robot.outtake.closeBothClaws();})
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
                    robot.intake.tiltDown();
                    robot.outtake.v4barScore();
                })
                .splineToSplineHeading(new Pose2d(15, 29, Math.toRadians(90)), Math.toRadians(-90))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.special.releasePixel();})
                .waitSeconds(1)


                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.outtake.v4barScore();})
                .splineToSplineHeading(new Pose2d(50, 36, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.outtake.openBothClaws();})
                .waitSeconds(1.5)


                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{robot.outtake.v4barStow();})
                .waitSeconds(0.5)
                .forward(3)
                .lineToLinearHeading(new Pose2d(45, 12, Math.toRadians(-155)))
                .build();
        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(startPose) // left
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.tiltUp();
                    robot.outtake.openBothClaws();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {robot.outtake.v4barTransfer();})
                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {robot.outtake.closeBothClaws();})
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
                    robot.intake.tiltDown();
                    robot.outtake.v4barScore();
                })
                .splineToSplineHeading(new Pose2d(23, 34, Math.toRadians(90)), Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.special.releasePixel();})
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.transfer();})
                .waitSeconds(1)


                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.outtake.v4barScore();})
                .splineToSplineHeading(new Pose2d(48, 42, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.outtake.openBothClaws();})
                .waitSeconds(1.5)


                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{robot.outtake.v4barStow();})
                .waitSeconds(0.5)
                .forward(3)
                .lineToLinearHeading(new Pose2d(45, 12, Math.toRadians(-155)))
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


        switch (loc) {
            case "left":
                robot.drive.followTrajectorySequence(left);
                break;
            case "right":
                robot.drive.followTrajectorySequence(middle);
                break;
            case "none":
                robot.drive.followTrajectorySequence(right);
                break;
        }
        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = robot.drive.getPoseEstimate();


    }
}
