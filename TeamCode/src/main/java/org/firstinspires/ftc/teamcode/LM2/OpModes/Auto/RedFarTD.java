package org.firstinspires.ftc.teamcode.LM2.OpModes.Auto;

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
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.Prop.RedFarPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled

@Config
@Autonomous
public class RedFarTD extends LinearOpMode {

    FtcDashboard dashboard;

    public static String loc = "left";
    MecanumDrive drive;
    LM2class robot;

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
        robot = new LM2class(hardwareMap);

        Pose2d startPose = new Pose2d(-38, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(startPose) // left
                .splineTo(new Vector2d(-36.6, -32), Math.toRadians(130))
                .build();
        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(startPose) // middle
                .splineToSplineHeading(new Pose2d(-40, -25, Math.toRadians(10)), Math.toRadians(10))
                .build();
        TrajectorySequence traj13 = drive.trajectorySequenceBuilder(startPose) // right
                .splineToSplineHeading(new Pose2d(-31, -34, Math.toRadians(30)), Math.toRadians(0))
                .build();
        TrajectorySequence traj21 = drive.trajectorySequenceBuilder(traj11.end())


                .splineToSplineHeading(new Pose2d(-33, -45, Math.toRadians(90)), Math.toRadians(-90))
//                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-33, -54, Math.toRadians(180)), Math.toRadians(-90))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(30, -54, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(51, -31.5, Math.toRadians(180)), Math.toRadians(0))

                //.splineTo(new Vector2d(-41, -32), Math.toRadians(150))
//                .lineToLinearHeading(new Pose2d(-34, -32, Math.toRadians(120)))
//                .lineToLinearHeading(new Pose2d(-34, -9, Math.toRadians(0)))
//                .setTangent(0)
//                .splineTo(new Vector2d(22, -9), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(55, -22, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj22 = drive.trajectorySequenceBuilder(traj12.end())
                .splineToSplineHeading(new Pose2d(-43, -55, Math.toRadians(0)), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-48, -22), Math.toRadians(180))
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-31, -9), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(22, -9, Math.toRadians(0)), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(55, -27.5, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj23 = drive.trajectorySequenceBuilder(traj13.end())
                //.splineTo(new Vector2d(-41, -32), Math.toRadians(150))
                .splineToSplineHeading(new Pose2d(-43, -55, Math.toRadians(0)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(30, -55, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(51, -31.5, Math.toRadians(0)), Math.toRadians(0))


//                .setTangent(Math.toRadians(180))
//                .splineToSplineHeading(new Pose2d(-38, -34, Math.toRadians(0)), Math.toRadians(-180))
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-31, -9.5), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(22, -9.5, Math.toRadians(0)), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(51, -31.5, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(traj21.end())
                .back(12)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(traj22.end())
                .back(12)
                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(traj23.end())
                .back(12)
                .build();

        robot.holdDrone();
        robot.closeClaw();
        while (opModeInInit()) {
            loc = redFarPropThreshold.getPropPosition();
            telemetry.addData("Prop Position", redFarPropThreshold.getPropPosition());
            telemetry.addData("Avg Left Value", redFarPropThreshold.getAvergageLeft());
            telemetry.addData("Avg Right Value", redFarPropThreshold.getAvergageRight());
            telemetry.update();
        }

        waitForStart();
//        sleep(12000);
        switch (loc) {
            case "none":
                drive.followTrajectorySequence(traj13);
                break;
            case "left":
                drive.followTrajectorySequence(traj11);
                break;
            case "right":
                drive.followTrajectorySequence(traj12);
                break;
        }

        robot.openLeft();
        sleep(3000);
        robot.setArm(75); // 390

        //outtake
        switch (loc) {
            case "none":
                drive.followTrajectorySequence(traj23);
                break;
            case "left":
                drive.followTrajectorySequence(traj21);
                break;
            case "right":
                drive.followTrajectorySequence(traj22);
                break;
        }

        //robot.openClaw();
        robot.openRight();
        sleep(500);

        switch (loc) {
            case "none":
                drive.followTrajectorySequence(park3);
                break;
            case "left":
                drive.followTrajectorySequence(park1);
                break;
            case "right":
                drive.followTrajectorySequence(park2);
                break;
        }
        robot.setArm(0); // 700

    }
}
