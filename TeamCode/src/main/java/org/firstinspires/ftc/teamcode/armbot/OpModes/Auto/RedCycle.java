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
@Autonomous
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

        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(startPose) //left
                .splineToSplineHeading(new Pose2d(11, -33, Math.toRadians(160)), Math.toRadians(130))
                .build();
        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(startPose) //mid
                .splineToSplineHeading(new Pose2d(15, -31, Math.toRadians(90)), Math.toRadians(90))
                .build();
        TrajectorySequence traj13 = drive.trajectorySequenceBuilder(startPose) //right
                .splineToSplineHeading(new Pose2d(15, -35, Math.toRadians(45)), Math.toRadians(90))
                .build();
        TrajectorySequence traj21 = drive.trajectorySequenceBuilder(traj11.end())
                .setTangent(Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(51.5, -24, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj22 = drive.trajectorySequenceBuilder(traj12.end())
                .setTangent(Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(52, -32, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence traj23 = drive.trajectorySequenceBuilder(traj13.end())
                .setTangent(Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(52, -39, Math.toRadians(0)), Math.toRadians(0))
                .build();

//        TrajectorySequence park1 = drive.trajectorySequenceBuilder(traj21.end())
//                .back(4)
//                .strafeRight(28)
//                .build();
//        TrajectorySequence park2 = drive.trajectorySequenceBuilder(traj22.end())
//                .back(4)
//                .strafeRight(22)
//                .build();
//        TrajectorySequence park3 = drive.trajectorySequenceBuilder(traj23.end())
//                .back(4)
//                .strafeRight(18)
//                .build();


        TrajectorySequence cycle1 = drive.trajectorySequenceBuilder(traj21.end())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(11.5, -9.5, Math.toRadians(-180)), Math.toRadians(-180))
                .build();
        TrajectorySequence cycle2 = drive.trajectorySequenceBuilder(traj22.end())
                .setTangent(Math.toRadians(160))
                .splineToSplineHeading(new Pose2d(11.5, -9.5, Math.toRadians(-180)), Math.toRadians(-180))
                .build();
        TrajectorySequence cycle3 = drive.trajectorySequenceBuilder(traj23.end())
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(new Pose2d(11.5, -9.5, Math.toRadians(-180)), Math.toRadians(-180))
                .build();

        TrajectorySequence stack = drive.trajectorySequenceBuilder(cycle1.end())
                .splineToSplineHeading(new Pose2d(-58, -9.5, Math.toRadians(-180)), Math.toRadians(-180))
                .build();

        TrajectorySequence score = drive.trajectorySequenceBuilder(stack.end())
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-20, -9.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(11.5, -9.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(51.5, -24), Math.toRadians(0))
                .build();

        TrajectorySequence back = drive.trajectorySequenceBuilder(score.end())
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
        //robot.setArm(-700);
        //sleep(1000);
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
        robot.openLeft();
        sleep(3000);
        robot.setArm(350); // 390

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

        robot.openClaw(); // Score Yellow Preload

        sleep(1000);
        // 700

        switch (loc) {
            case "left":
                drive.followTrajectorySequence(cycle1);
                break;
            case "center":
                drive.followTrajectorySequence(cycle2);
                break;
            case "right":
                drive.followTrajectorySequence(cycle3);
                break;
        }
        robot.setArm(0);

        drive.followTrajectorySequence(stack);
        robot.closeClaw();
        sleep(1000);
        drive.followTrajectorySequence(score);
        robot.openClaw();
        sleep(1000);
        drive.followTrajectorySequence(back);

    }
}
