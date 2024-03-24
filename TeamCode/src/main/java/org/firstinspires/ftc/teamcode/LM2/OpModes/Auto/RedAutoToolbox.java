package org.firstinspires.ftc.teamcode.LM2.OpModes.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.Prop.RedPropThreshold;
import org.firstinspires.ftc.teamcode.LM2.LM2class;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous
public class RedAutoToolbox extends LinearOpMode {

    FtcDashboard dashboard;

    public static String loc = "left";
    MecanumDrive drive;
    LM2class robot;

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
        robot = new LM2class(hardwareMap);

        Pose2d startPose = new Pose2d(15, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(startPose) //left
                .splineToSplineHeading(new Pose2d(9, -25, Math.toRadians(160)), Math.toRadians(130))
                .build();
        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(startPose) //mid
                .splineToSplineHeading(new Pose2d(15, -30, Math.toRadians(90)), Math.toRadians(90))
                .build();
        TrajectorySequence traj13 = drive.trajectorySequenceBuilder(startPose) //right
                .splineToSplineHeading(new Pose2d(15, -29, Math.toRadians(45)), Math.toRadians(90))
                .build();
        TrajectorySequence traj21 = drive.trajectorySequenceBuilder(traj11.end())
                .setTangent(Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(51.5, -21, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj22 = drive.trajectorySequenceBuilder(traj12.end())
                .setTangent(Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(53, -30, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj23 = drive.trajectorySequenceBuilder(traj13.end())
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(54, -33, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(traj21.end())
                .back(4)
                .strafeLeft(16)
//                .strafeRight(34)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.setArm(0);}) // score purple Preload

//                .forward(15)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(traj22.end())
                .back(4)
                .strafeLeft(24)

//                .strafeRight(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.setArm(0);}) // score purple Preload

//                .forward(15)

                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(traj23.end())
                .back(4)
                .strafeLeft(28)

//                .strafeRight(28)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {robot.setArm(0);}) // score purple Preload

//                .forward(15)

                .build();

        robot.closeDrone();
        robot.closeClaw();
        while (opModeInInit()) {
            loc = redPropThreshold.getPropPosition();
            telemetry.addData("Prop Position", redPropThreshold.getPropPosition());
            telemetry.addData("Avg Left Value", redPropThreshold.getAvergageLeft());
            telemetry.addData("Avg Right Value", redPropThreshold.getAvergageRight());
            telemetry.update();
        }

        waitForStart();
        //robot.setArm(-700);
        //sleep(1000);
        switch (loc) {
            case "none":
                drive.followTrajectorySequence(traj11);
                break;
            case "left":
                drive.followTrajectorySequence(traj12);
                break;
            case "right":
                drive.followTrajectorySequence(traj13);
                break;
        }

        //robot.openAutoClaw();
        robot.openLeft();
        sleep(200);
        robot.setArm(330); // 390

        //outtake
        switch (loc) {
            case "none":
                drive.followTrajectorySequence(traj21);
                break;
            case "left":
                drive.followTrajectorySequence(traj22);
                break;
            case "right":
                drive.followTrajectorySequence(traj23);
                break;
        }

        robot.openClaw();

        sleep(250);
        // 700

        switch (loc) {
            case "none":
                drive.followTrajectorySequence(park1);
                break;
            case "left":
                drive.followTrajectorySequence(park2);
                break;
            case "right":
                drive.followTrajectorySequence(park3);
                break;
        }
        robot.setArm(0);

    }
}
