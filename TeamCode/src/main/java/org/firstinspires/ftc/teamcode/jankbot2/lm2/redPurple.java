package org.firstinspires.ftc.teamcode.jankbot2.lm2;

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
import org.firstinspires.ftc.teamcode.Vision.Prop.RedFarPropThreshold;
import org.firstinspires.ftc.teamcode.Vision.Prop.RedPropThreshold;
import org.firstinspires.ftc.teamcode.jankbot2.jankbot2Class;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous
public class redPurple extends LinearOpMode {

    FtcDashboard dashboard;

    public static String loc = "left";
    MecanumDrive drive;
    public static jankbot2Class robot;

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
        robot = new jankbot2Class(hardwareMap);

        Pose2d startPose = new Pose2d(-40, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-36, -23, Math.toRadians(50)), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    robot.setArm(-904);
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    robot.openClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    robot.setArm(-800);
                })
                .forward(3)

                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-40, -14, Math.toRadians(90)), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    robot.setArm(-904);
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    robot.openClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    robot.setArm(-800);
                })
                .forward(3)

                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)

                .splineToSplineHeading(new Pose2d(-48, -32, Math.toRadians(180)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    robot.setArm(-830);
                })
                .waitSeconds(2)
                .splineToSplineHeading(new Pose2d(-42, -32, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    robot.openClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    robot.setArm(-800);
                })
                .forward(3)
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
                drive.followTrajectorySequence(right);
                break;
            case "center":
                drive.followTrajectorySequence(left);
                break;
            case "right":
                drive.followTrajectorySequence(middle);
                break;
        }


    }
}