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
import org.firstinspires.ftc.teamcode.Vision.Prop.RedFarPropThreshold;
import org.firstinspires.ftc.teamcode.jankbot.Jankbot;
import org.firstinspires.ftc.teamcode.jankbot.competition.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Disabled
@Autonomous (group = "Blue", preselectTeleOp = "JankTele")
public class RedFar extends LinearOpMode {
    FtcDashboard dashboard;

    public static String loc = "left";
    public static boolean middlePark = false;
    Jankbot robot;

    private VisionPortal portal;
    private RedFarPropThreshold redFarPropThreshold;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        redFarPropThreshold = new RedFarPropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(redFarPropThreshold)
                .build();


        robot = new Jankbot(hardwareMap);

        Pose2d startPose = new Pose2d(-40, -63.5, Math.toRadians(-90));
        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(startPose) //
                // Drive to spike
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(8, -34, Math.toRadians(180)), Math.toRadians(150))


                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.tiltDown();
                    robot.intake.setIntake(-0.15);
                })
                .waitSeconds(1) // Score Purple Spike
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.tiltStow();
                    robot.intake.setIntake(0);
                })


                // Drive to Board
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(43, 28, Math.toRadians(180)), Math.toRadians(0))


                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.outtake.v4barScore();
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.outtake.openBothClaws();
                })
                .waitSeconds(2.5) // Place Yellow
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.v4barTransfer();
                })

                .build();
        TrajectorySequence middle = robot.drive.trajectorySequenceBuilder(startPose) // middle
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(16.5, -28, Math.toRadians(150)), Math.toRadians(90))


                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.tiltDown();
                    robot.intake.setIntake(-0.15);
                })
                .waitSeconds(1) // Score Purple Spike
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.tiltStow();
                    robot.intake.setIntake(0);
                })



                .setReversed(true)
                .splineToSplineHeading(new Pose2d(43, -35, Math.toRadians(180)), Math.toRadians(0))



                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.outtake.v4barScore();
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.outtake.openBothClaws();
                })
                .waitSeconds(2.5) // Place Yellow
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.v4barTransfer();
                })



                .build();
        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startPose) // Truss side / no prop seen
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-31, -31, Math.toRadians(180)), Math.toRadians(45))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.special.releasePixel();
                })
                .waitSeconds(0.5) // Score Purple Spike


                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-60, -11.5), Math.toRadians(180))
//                                .splineToSplineHeading(new Pose2d(-60, -11.5, Math.toRadians(180)), Math.toRadians(180))


                .waitSeconds(1) // Pick from stack


                // Drive to Board
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(40, -11.5, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(0.2) // Wat for robot
                .lineToLinearHeading(new Pose2d(46, -42, Math.toRadians(180)))
                .waitSeconds(.3)
                .lineToLinearHeading(new Pose2d(47, -43, Math.toRadians(180)))

                .waitSeconds(1.5) // Place Yellow
                .build();





        // On Init
        robot.special.holdPixel();
        robot.intake.tiltStow();
        robot.outtake.v4barStow();
        robot.outtake.openBothClaws();






        while (opModeInInit()) {
            loc = redFarPropThreshold.getPropPosition();
            telemetry.addData("Prop Position", redFarPropThreshold.getPropPosition());
            telemetry.addData("Avg Left Box Value", redFarPropThreshold.getAvergageLeft());
            telemetry.addData("Avg Right Box Value", redFarPropThreshold.getAvergageRight());

            if (gamepad1.a) {
                middlePark = true;
            } else if (gamepad1.b) {
                middlePark = false;
            }
            telemetry.addData("Ideal Park?", middlePark);

            telemetry.update();
        }


        waitForStart();


        switch (loc) {
            case "left":
                robot.drive.followTrajectorySequence(left);
                break;
            case "right":
                robot.drive.followTrajectorySequence(middle);
                break;
            case "none":
                robot.drive.followTrajectorySequence(right);
                break;
        }

        if (middlePark) {
            TrajectorySequence idealPark = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()) // Ideal Park pointed towards truss
                    .setReversed(false)
                    .lineToLinearHeading(new Pose2d(43, -13, Math.toRadians(155)))
                    .build();
            robot.drive.followTrajectorySequence(idealPark);
        } else {
            TrajectorySequence cornerPark = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()) // Corner Park
                    .setReversed(false)
                    .lineToLinearHeading(new Pose2d(46, -58, Math.toRadians(125))) // Corner Park
                    .build();
            robot.drive.followTrajectorySequence(cornerPark);
        }



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = robot.drive.getPoseEstimate();


    }
}
