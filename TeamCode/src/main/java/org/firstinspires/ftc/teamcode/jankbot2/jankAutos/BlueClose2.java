package org.firstinspires.ftc.teamcode.jankbot2.jankAutos;

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
import org.firstinspires.ftc.teamcode.jankbot.Jankbot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous (group = "Blue", preselectTeleOp = "JankTele")
public class BlueClose2 extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(17, 63.5, Math.toRadians(90));
        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startPose) // right side
                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0.75, ()->{robot.jankOuttake.armDown();})
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.jankOuttake.closeBoth();})
                .splineToSplineHeading(new Pose2d(10,40, Math.toRadians(45)), Math.toRadians(-120))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.jankOuttake.openLeft();})
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{robot.jankOuttake.scoreArm();})
                .waitSeconds(1.5)
                .splineToSplineHeading(new Pose2d(48, 42, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{robot.jankOuttake.openRight();})
                .forward(4)
                .strafeLeft(22)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.intake.tiltUp();
//                    robot.outtake.openBothClaw();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {robot.outtake.v4barTransfer();})
//                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {robot.outtake.closeBothClaw();})
//                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
//                    robot.intake.tiltDown();
//                    robot.outtake.v4barScore();
//                })
//                .splineToSplineHeading(new Pose2d(5, 36, Math.toRadians(30)), Math.toRadians(-130))
////                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.special.releasePixel();})
//                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.transfer();})
//                .waitSeconds(1) // Score Purple
//
//
//
//                .setReversed(false)
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.outtake.v4barScore();})
//                .splineToSplineHeading(new Pose2d(50, 42, Math.toRadians(180)), Math.toRadians(0))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.outtake.openBothClaw();})
//                .waitSeconds(1.5)
//
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{robot.outtake.v4barStow();})
//                .waitSeconds(0.5)
//                .forward(3)
//                .lineToLinearHeading(new Pose2d(45, 12, Math.toRadians(-155)))
                .build();
        TrajectorySequence middle = robot.drive.trajectorySequenceBuilder(startPose) // middle
                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0.75, ()->{robot.jankOuttake.armDown();})
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.jankOuttake.closeBoth();})
                .splineToSplineHeading(new Pose2d(16,35, Math.toRadians(90)), Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.jankOuttake.openLeft();})
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{robot.jankOuttake.scoreArm();})
                .waitSeconds(1.5)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(48, 36, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{robot.jankOuttake.openRight();})
                .forward(4)
                .strafeLeft(16)
//                .setReversed(true)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.intake.tiltUp();
//                    robot.outtake.openBothClaw();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {robot.outtake.v4barTransfer();})
//                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {robot.outtake.closeBothClaw();})
//                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
//                    robot.intake.tiltDown();
//                    robot.outtake.v4barScore();
//                })
//                .splineToSplineHeading(new Pose2d(15, 29, Math.toRadians(90)), Math.toRadians(-90))
////                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.special.releasePixel();})
//                .waitSeconds(1)
//
//
//                .setReversed(false)
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.outtake.v4barScore();})
//                .splineToSplineHeading(new Pose2d(50, 36, Math.toRadians(180)), Math.toRadians(0))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.outtake.openBothClaw();})
//                .waitSeconds(1.5)
//
//
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{robot.outtake.v4barStow();})
//                .waitSeconds(0.5)
//                .forward(3)
//                .lineToLinearHeading(new Pose2d(45, 12, Math.toRadians(-155)))
                .build();
        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(startPose) // left
                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0.75, ()->{robot.jankOuttake.armDown();})
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.jankOuttake.closeBoth();})
                .splineToSplineHeading(new Pose2d(18,40, Math.toRadians(135)), Math.toRadians(-60))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.jankOuttake.openLeft();})
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{robot.jankOuttake.scoreArm();})
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(10, 24, Math.toRadians(180)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(48, 42, Math.toRadians(180)), Math.toRadians(60))
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{robot.jankOuttake.openRight();})
                .forward(4)
                .strafeRight(10)
//                .setReversed(true)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.intake.tiltUp();
//                    robot.outtake.openBothClaw();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {robot.outtake.v4barTransfer();})
//                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {robot.outtake.closeBothClaw();})
//                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
//                    robot.intake.tiltDown();
//                    robot.outtake.v4barScore();
//                })
//                .splineToSplineHeading(new Pose2d(23, 34, Math.toRadians(90)), Math.toRadians(-90))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.special.releasePixel();})
//                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.transfer();})
//                .waitSeconds(1)
//
//
//                .setReversed(false)
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.outtake.v4barScore();})
//                .splineToSplineHeading(new Pose2d(48, 42, Math.toRadians(180)), Math.toRadians(0))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.outtake.openBothClaw();})
//                .waitSeconds(1.5)
//
//
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{robot.outtake.v4barStow();})
//                .waitSeconds(0.5)
//                .forward(3)
//                .lineToLinearHeading(new Pose2d(45, 12, Math.toRadians(-155)))
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
            case "right":
                robot.drive.followTrajectorySequence(right);
                break;
            case "center":
                robot.drive.followTrajectorySequence(middle);
                break;
            case "left":
                robot.drive.followTrajectorySequence(left);
                break;
        }

    }
}
