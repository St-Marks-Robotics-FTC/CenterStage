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
import org.firstinspires.ftc.teamcode.Vision.RedFarPropThreshold;
import org.firstinspires.ftc.teamcode.armbot.BozoClass;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous
public class RedFarAutoToolbox extends LinearOpMode {

    FtcDashboard dashboard;

    public static String loc = "left";
    MecanumDrive drive;
    BozoClass robot;

    private VisionPortal portal;
    private RedFarPropThreshold redFarPropThreshold;


    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        redFarPropThreshold = new RedFarPropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(redFarPropThreshold)
                .build();



        drive = new MecanumDrive(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(12, -60, Math.toRadians(90)));
        robot = new BozoClass(hardwareMap);

        Pose2d startPose = new Pose2d(-38, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(startPose) // left
                .splineTo(new Vector2d(-41, -32), Math.toRadians(150))
                .build();
        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(startPose) // middle
                .splineToSplineHeading(new Pose2d(-40, -26, Math.toRadians(10)), Math.toRadians(10))
                .build();
        TrajectorySequence traj13 = drive.trajectorySequenceBuilder(startPose) // right
                .splineToSplineHeading(new Pose2d(-31, -34, Math.toRadians(30)), Math.toRadians(0))
                .build();
        TrajectorySequence traj21 = drive.trajectorySequenceBuilder(traj11.end())
                //.splineTo(new Vector2d(-41, -32), Math.toRadians(150))
                .lineToLinearHeading(new Pose2d(-34, -32, Math.toRadians(120)))
                .lineToLinearHeading(new Pose2d(-34, -11, Math.toRadians(90)))
                .setTangent(0)
                .splineTo(new Vector2d(11, -11), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(50, -27, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj22 = drive.trajectorySequenceBuilder(traj12.end())
                .splineToConstantHeading(new Vector2d(-45, -26), Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-31, -9), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(11, -9, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(50, -36, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj23 = drive.trajectorySequenceBuilder(traj13.end())
                .splineToSplineHeading(new Pose2d(-38, -34, Math.toRadians(0)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-31, -11), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(11, -11, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(50, -40, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(traj21.end())
                .strafeLeft(18)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(traj22.end())
                .strafeLeft(22)
                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(traj23.end())
                .strafeLeft(28)
                .build();

        //robot.closeClaw();
        while (opModeInInit()) {
            loc = redFarPropThreshold.getPropPosition();
            telemetry.addData("Prop Position", redFarPropThreshold.getPropPosition());
            telemetry.addData("Avg Left Value", redFarPropThreshold.getAvergageLeft());
            telemetry.addData("Avg Right Value", redFarPropThreshold.getAvergageRight());
            telemetry.update();
        }

        waitForStart();
        robot.setArm(-700);
        sleep(1000);
        switch (loc) {
            case "left":
                drive.followTrajectorySequence(traj11);
                break;
            case "center":
                drive.followTrajectorySequence(traj12);
                break;
            case "right":
                drive.followTrajectorySequence(traj13);
                break;
        }

        //robot.openAutoClaw();
        sleep(3000);
        robot.setArm(-1650); // 390

        //outtake
        switch (loc) {
            case "left":
                drive.followTrajectorySequence(traj21);
                break;
            case "center":
                drive.followTrajectorySequence(traj22);
                break;
            case "right":
                drive.followTrajectorySequence(traj23);
                break;
        }

        //robot.openClaw();

        sleep(1000);
        robot.setArm(-800); // 700

        switch (loc) {
            case "left":
                drive.followTrajectorySequence(park1);
                break;
            case "center":
                drive.followTrajectorySequence(park2);
                break;
            case "right":
                drive.followTrajectorySequence(park3);
                break;
        }
        robot.setArm(-1600); // 700
        sleep(1000);

    }
}