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

        Pose2d startPose = new Pose2d(15, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(10.3, -29, Math.toRadians(160)), Math.toRadians(130))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.openLeft();}) // score purple Preload
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})


                .setTangent(Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(52, -22.5, Math.toRadians(0)), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score yellow Preload
                .waitSeconds(1.5)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.setArm(80);})
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(11.5, -8, Math.toRadians(-180)), Math.toRadians(-180)) // going to stack
                .splineToSplineHeading(new Pose2d(-11.5, -8, Math.toRadians(-180)), Math.toRadians(-180))

                .splineToSplineHeading(new Pose2d(-60, -6, Math.toRadians(-180)), Math.toRadians(-180)) // stack
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(1.5)


                // stack to board
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-20, -9.5, Math.toRadians(0 + 0.000001)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(11.5, -9.5, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(390);})
                .splineToConstantHeading(new Vector2d(52.5, -26), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d( 53, -32, Math.toRadians(0)))
                .waitSeconds(1)

                .back(4)

                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(15, -31, Math.toRadians(90)), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.openLeft();}) // score purple Preload
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})


                .setTangent(Math.toRadians(20))
                .splineToSplineHeading(new Pose2d(51.5, -30, Math.toRadians(0)), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score yellow Preload
                .waitSeconds(1.5)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.setArm(80);})
                .setTangent(Math.toRadians(-160))
                .splineToSplineHeading(new Pose2d(11.5, -8, Math.toRadians(-180)), Math.toRadians(-180)) // going to stack
                .splineToSplineHeading(new Pose2d(-11.5, -8, Math.toRadians(-180)), Math.toRadians(-180))

                .splineToSplineHeading(new Pose2d(-60.5, -3.5, Math.toRadians(-180)), Math.toRadians(-180)) // stack
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(1.5)


                // stack to board
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-20, -7, Math.toRadians(0 + 0.000001)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(11.5, -5.5, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(370);})
                .splineToConstantHeading(new Vector2d(52, -24), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d( 52, -32, Math.toRadians(0)))
                .waitSeconds(1)
                .back(4)

                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(15, -35, Math.toRadians(45)), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.openLeft();
                }) // score purple Preload
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})


                .setTangent(Math.toRadians(-35))
                .splineToSplineHeading(new Pose2d(54, -36, Math.toRadians(0)), Math.toRadians(0))


                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();}) // score yellow Preload
                .waitSeconds(1.5)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.setArm(70);})
                .setTangent(Math.toRadians(-140))
                .splineToSplineHeading(new Pose2d(11.5, -8, Math.toRadians(-180)), Math.toRadians(-180)) // going to stack
                .splineToSplineHeading(new Pose2d(-11.5, -8, Math.toRadians(-180)), Math.toRadians(-180))

                .splineToSplineHeading(new Pose2d(-62, -5, Math.toRadians(-180)), Math.toRadians(-180)) // stack
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.closeClaw();})
                .waitSeconds(1.5)


                // stack to board
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-20, -9, Math.toRadians(0 + 0.000001)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(11.5, -9, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.setArm(350);})
                .splineToConstantHeading(new Vector2d(51.5, -24), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.openClaw();})
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d( 50, -32, Math.toRadians(0)))
                .waitSeconds(1)

                .back(4)

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
                drive.followTrajectorySequence(left);
                break;
            case "right":
                drive.followTrajectorySequence(right);
                break;
            case "left":
                drive.followTrajectorySequence(middle);
                break;
        }
    }
}
