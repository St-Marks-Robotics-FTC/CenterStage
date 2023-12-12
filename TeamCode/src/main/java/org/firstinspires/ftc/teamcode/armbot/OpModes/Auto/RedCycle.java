package org.firstinspires.ftc.teamcode.armbot.OpModes.Auto;

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
import org.firstinspires.ftc.teamcode.Vision.RedPropThreshold;
import org.firstinspires.ftc.teamcode.armbot.BozoClass;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous (group = "Red")
public class RedCycle extends LinearOpMode {

    FtcDashboard dashboard;

    public static String loc = "left";
    MecanumDrive drive;
    BozoClass robot;

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
        robot = new BozoClass(hardwareMap);

        Pose2d startPose = new Pose2d(15, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(11, -33, Math.toRadians(160)), Math.toRadians(130))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.openLeft();}) // score purple Preload
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})


                .setTangent(Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(51.5, -24, Math.toRadians(0)), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score yellow Preload
                .waitSeconds(1.5)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.setArm(0);})
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(11.5, -9.5, Math.toRadians(-180)), Math.toRadians(-180))

                .splineToSplineHeading(new Pose2d(-58, -9.5, Math.toRadians(-180)), Math.toRadians(-180)) // stack
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(1.5)


                // stack to board
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-20, -9.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(16, -9.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(51.5, -31), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(1.5)

                .back(4)

                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(15, -31, Math.toRadians(90)), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.openLeft();}) // score purple Preload
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})


                .setTangent(Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(52, -32, Math.toRadians(0)), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score yellow Preload
                .waitSeconds(1.5)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.setArm(0);})
                .setTangent(Math.toRadians(160))
                .splineToSplineHeading(new Pose2d(11.5, -9.5, Math.toRadians(-180)), Math.toRadians(-180))

                .splineToSplineHeading(new Pose2d(-58, -9.5, Math.toRadians(-180)), Math.toRadians(-180)) // stack
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(1.5)


                // stack to board
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-20, -9.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(16, -9.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(51.5, -31), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(1.5)

                .back(4)

                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(15, -35, Math.toRadians(45)), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.openLeft();}) // score purple Preload
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})


                .setTangent(Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(52, -39, Math.toRadians(0)), Math.toRadians(0))


                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score yellow Preload
                .waitSeconds(1.5)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.setArm(0);})
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(new Pose2d(11.5, -9.5, Math.toRadians(-180)), Math.toRadians(-180))

                .splineToSplineHeading(new Pose2d(-58, -9.5, Math.toRadians(-180)), Math.toRadians(-180)) // stack
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(1.5)


                // stack to board
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-20, -9.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(16, -9.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(51.5, -31), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score yellow Preload
                .waitSeconds(1.5)

                .back(4)

                .build();



        robot.closeClaw();
        while (opModeInInit()) {
            loc = redPropThreshold.getPropPosition();
            telemetry.addData("Prop Position", redPropThreshold.getPropPosition());
            telemetry.addData("Avg Left Value", redPropThreshold.getAvergageLeft());
            telemetry.addData("Avg Right Value", redPropThreshold.getAvergageRight());
            telemetry.update();
        }

        waitForStart();

        switch (loc) {
            case "left":
                drive.followTrajectorySequence(left);
                break;
            case "center":
                drive.followTrajectorySequence(middle);
                break;
            case "right":
                drive.followTrajectorySequence(right);
                break;
        }
    }
}
