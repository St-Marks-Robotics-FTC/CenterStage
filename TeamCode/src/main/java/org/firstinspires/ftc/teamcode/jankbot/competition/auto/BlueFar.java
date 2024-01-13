package org.firstinspires.ftc.teamcode.jankbot.competition.auto;

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
import org.firstinspires.ftc.teamcode.Vision.Prop.BlueFarPropThreshold;
import org.firstinspires.ftc.teamcode.Vision.Prop.BluePropThreshold;
import org.firstinspires.ftc.teamcode.jankbot.Jankbot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Disabled
@Autonomous (group = "Blue", preselectTeleOp = "JankTele")
public class BlueFar extends LinearOpMode {
    FtcDashboard dashboard;

    public static String loc = "left";
    //MecanumDrive drive;
    Jankbot robot;

    private VisionPortal portal;
    private BlueFarPropThreshold bluePropThreshold;
    private static int waitTime = 10000; //milliseconds


    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bluePropThreshold = new BlueFarPropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(bluePropThreshold)
                .build();



        //drive = new MecanumDrive(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(12, -60, Math.toRadians(90)));
        robot = new Jankbot(hardwareMap);

        Pose2d startPose = new Pose2d(-41, 60, Math.toRadians(90));
        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startPose) // right side
                .setReversed(true)
                .setTangent(Math.toRadians(-80))
                .UNSTABLE_addTemporalMarkerOffset(0.75, ()->{robot.jankOuttake.armDown();})
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.jankOuttake.closeBoth();})
                .splineToSplineHeading(new Pose2d(-42,40, Math.toRadians(60)), Math.toRadians(-120))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.jankOuttake.openLeft();})
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{robot.jankOuttake.scoreArm();})
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-32, 24, Math.toRadians(180)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-32, 12, Math.toRadians(180)), Math.toRadians(-90))
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(20, 12, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(48, 30, Math.toRadians(180)), Math.toRadians(60))
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{robot.jankOuttake.openRight();})
                .forward(4)
                .strafeLeft(22)
                .build();
        TrajectorySequence middle = robot.drive.trajectorySequenceBuilder(startPose) // middle
                .setReversed(true)
                .setTangent(Math.toRadians(-80))
                .UNSTABLE_addTemporalMarkerOffset(0.75, ()->{robot.jankOuttake.armDown();})
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.jankOuttake.closeBoth();})
                .splineToSplineHeading(new Pose2d(-42,40, Math.toRadians(60)), Math.toRadians(-120))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.jankOuttake.openLeft();})
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{robot.jankOuttake.scoreArm();})
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-32, 24, Math.toRadians(180)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-32, 12, Math.toRadians(180)), Math.toRadians(-90))
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(20, 12, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(48, 30, Math.toRadians(180)), Math.toRadians(60))
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{robot.jankOuttake.openRight();})
                .forward(4)
                .strafeLeft(22)
                .build();
        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(startPose) // left
                .setReversed(true)
                .setTangent(Math.toRadians(-80))
                .UNSTABLE_addTemporalMarkerOffset(0.75, ()->{robot.jankOuttake.armDown();})
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.jankOuttake.closeBoth();})
                .splineToSplineHeading(new Pose2d(-42,40, Math.toRadians(60)), Math.toRadians(-120))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.jankOuttake.openLeft();})
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{robot.jankOuttake.scoreArm();})
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-32, 24, Math.toRadians(180)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-32, 12, Math.toRadians(180)), Math.toRadians(-90))
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(20, 12, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(48, 30, Math.toRadians(180)), Math.toRadians(60))
                .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{robot.jankOuttake.openRight();})
                .forward(4)
                .strafeLeft(22)
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
        //robot.special.grabPixel();
        sleep(waitTime);
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
