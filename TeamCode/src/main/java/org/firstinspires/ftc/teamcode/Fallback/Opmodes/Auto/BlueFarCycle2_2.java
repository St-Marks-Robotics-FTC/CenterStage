package org.firstinspires.ftc.teamcode.Fallback.Opmodes.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Fallback.FallbackClass;
import org.firstinspires.ftc.teamcode.Vision.Prop.BlueFarPropThreshold;
//import org.firstinspires.ftc.teamcode.jankbot.Jankbot;
//import org.firstinspires.ftc.teamcode.jankbot.competition.PoseStorage;
import org.firstinspires.ftc.teamcode.Fallback.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous (group = "Blue", preselectTeleOp = "JankTele")
public class BlueFarCycle2_2 extends LinearOpMode {
    FtcDashboard dashboard;

    public static String loc = "left";
    public static boolean middlePark = false;
    FallbackClass robot;

    private VisionPortal portal;
    private BlueFarPropThreshold blueFarPropThreshold;
    private int delay = 10000;
    private double startY = 0.0;
    private double depositY=0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        blueFarPropThreshold = new BlueFarPropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(blueFarPropThreshold)
                .build();


        robot = new FallbackClass(hardwareMap);

        Pose2d startPose = new Pose2d(-41, 63.5, Math.toRadians(90));
        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(startPose) // Truss side / No Prop Seen
                // Drive to spike
                .setReversed(true)
                .setTangent(Math.toRadians(-120))
                .splineToLinearHeading(new Pose2d(-33, 36, Math.toRadians(-30)), Math.toRadians(0))

//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.openLeftClaw();
//                })
                .waitSeconds(1) // Score Purple Spike

                //Intake a White
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-65, 10, Math.toRadians(180)), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    robot.closeLeftClaw();
//                })

                .waitSeconds(1)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-26, 10, Math.toRadians(180)), Math.toRadians(0))

                .splineToConstantHeading(new Vector2d(10, 10) , Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(43, 42), Math.toRadians(0))

//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    robot.v4barScore();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.openClaw();
//                })
                .waitSeconds(1) // Place Yellow

                .build();
        TrajectorySequence middle = robot.drive.trajectorySequenceBuilder(startPose) // middle
                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-37, 24, Math.toRadians(0)), Math.toRadians(0))

//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.openLeftClaw();
//                })
                .waitSeconds(1) // Score Purple Spike

                //Intake a White
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-65, 10, Math.toRadians(180)), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    robot.closeLeftClaw();
//                })

                .waitSeconds(1)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-26, 10, Math.toRadians(180)), Math.toRadians(0))

                .splineToConstantHeading(new Vector2d(10, 10) , Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(43, 36), Math.toRadians(0))

//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    robot.v4barScore();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.openClaw();
//                })
                .waitSeconds(2.5) // Place Yellow



                .build();
        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startPose) // left
                .setTangent(Math.toRadians(-120))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-52, 18, Math.toRadians(60)), Math.toRadians(-120))

//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.openLeftClaw();
//                })
                .waitSeconds(1) // Score Purple Spike

                //Intake a White
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-65, 10, Math.toRadians(180)), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    robot.closeLeftClaw();
//                })

                .waitSeconds(1)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-26, 10, Math.toRadians(180)), Math.toRadians(0))

                .splineToConstantHeading(new Vector2d(10, 10) , Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(43, 30), Math.toRadians(0))


//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    robot.v4barScore();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.openClaw();
//                })
                .waitSeconds(2.5) // Place Yellow
                .build();

        // On Init
        //robot.closeClaw();

        while (opModeInInit()) {
            loc = blueFarPropThreshold.getPropPosition();
            telemetry.addData("Prop Position", blueFarPropThreshold.getPropPosition());
            telemetry.addData("Avg Left Box Value", blueFarPropThreshold.getAvergageLeft());
            telemetry.addData("Avg Right Box Value", blueFarPropThreshold.getAvergageRight());

            if (gamepad1.a) {
                middlePark = true;
            } else if (gamepad1.b) {
                middlePark = false;
            }
            telemetry.addData("Ideal Park?", middlePark);

            telemetry.update();
        }


        waitForStart();
        ElapsedTime time = new ElapsedTime();
        //sleep(delay);
        //robot.v4barPickup();
        switch (loc) {
            case "left":
                robot.drive.followTrajectorySequence(right);
                depositY=24;
                break;
            case "right":
                robot.drive.followTrajectorySequence(middle);
                depositY=24;
                break;
            case "none":
                robot.drive.followTrajectorySequence(left);
                depositY=30;
                break;
        }
        int cycles = 0;
        while (time.milliseconds()>8000 && cycles<1) {
            cycles++;
            TrajectorySequence cycle = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .setReversed(true)
                    .setTangent(Math.toRadians(-120))
                    .splineToSplineHeading(new Pose2d(15, 10,Math.toRadians(180)), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-65, 10, Math.toRadians(180)), Math.toRadians(180))
                    .waitSeconds(0.5)
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(15, 10,Math.toRadians(180)), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(43, 32,Math.toRadians(180)), Math.toRadians(45))
                    .waitSeconds(0.5)
                    .build();
            robot.drive.followTrajectorySequence(cycle);
        }

        if (middlePark) {
            TrajectorySequence idealPark = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()) // Ideal Park pointed towards truss
                    .setReversed(false)
                    .lineToLinearHeading(new Pose2d(43, 13, Math.toRadians(-155)))
                    .build();
            robot.drive.followTrajectorySequence(idealPark);
        } else {
            TrajectorySequence cornerPark = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()) // Corner Park
                    .setReversed(false)
                    .lineToLinearHeading(new Pose2d(46, 58, Math.toRadians(-125))) // Corner Park
                    .build();
            robot.drive.followTrajectorySequence(cornerPark);
        }



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
//        PoseStorage.currentPose = robot.drive.getPoseEstimate();


    }
}