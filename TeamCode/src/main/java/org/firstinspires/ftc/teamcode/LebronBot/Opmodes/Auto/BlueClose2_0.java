package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.Prop.BluePropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Disabled
@Autonomous (group = "Blue", preselectTeleOp = "JankTele")
public class BlueClose2_0 extends LinearOpMode {
    FtcDashboard dashboard;

    public static String loc = "left";
    public static boolean middlePark = false;
    LebronClass robot;

    private VisionPortal portal;
    private BluePropThreshold bluePropThreshold;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bluePropThreshold = new BluePropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(bluePropThreshold)
                .build();


        robot = new LebronClass(hardwareMap);

        Pose2d startPose = new Pose2d(16.5, 63.5, Math.toRadians(90));
        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startPose) // Truss side / No Prop Seen
                // Drive to spike
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(13, 34, Math.toRadians(0)), Math.toRadians(-150))


                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.outtake.v4barPurple();
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.outtake.v4BarAnglePurple();
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    robot.outtake.openRight();
                })
                .waitSeconds(4) // Score Purple Spike
                .UNSTABLE_addTemporalMarkerOffset( 0, () -> {
                    robot.outtake.v4barScore();
                    robot.outtake.v4barAngleScore();
                })

                // Drive to Board
                .setTangent(Math.toRadians(30))
                .splineToSplineHeading(new Pose2d(46, 28, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.openLeft();
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.outtake.v4barStow();
                    robot.outtake.v4barAngleStow();
                })

//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    robot.v4barScore();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.openClaw();
//                })
                .waitSeconds(2.5) // Place Yellow

                .build();
        TrajectorySequence middle = robot.drive.trajectorySequenceBuilder(startPose) // middle
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(16.5, 28, Math.toRadians(30)), Math.toRadians(-90))


//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.openLeftClaw();
//                })
                .waitSeconds(1) // Score Purple Spike

                .setReversed(true)
                .splineToSplineHeading(new Pose2d(48, 35, Math.toRadians(180)), Math.toRadians(0))

//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    robot.v4barScore();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.openClaw();
//                })
                .waitSeconds(2.5) // Place Yellow



                .build();
        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(startPose) // left
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(28, 36, Math.toRadians(45)), Math.toRadians(-45))


//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.openLeftClaw();
//                })
                .waitSeconds(1) // Score Purple Spike



                .setReversed(true)
                .splineToSplineHeading(new Pose2d(48, 42, Math.toRadians(180)), Math.toRadians(0))


//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    robot.v4barScore();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.openClaw();
//                })
                .waitSeconds(2.5) // Place Yellow
                .build();





        // On Init
        robot.outtake.closeBothClaws();
        robot.outtake.turretTransfer();
        robot.intake.tiltUp();

        while (opModeInInit()) {
//            loc = bluePropThreshold.getPropPosition();
//            telemetry.addData("Prop Position", bluePropThreshold.getPropPosition());
//            telemetry.addData("Avg Left Box Value", bluePropThreshold.getAvergageLeft());
//            telemetry.addData("Avg Right Box Value", bluePropThreshold.getAvergageRight());

            if (gamepad1.a) {
                middlePark = true;
            } else if (gamepad1.b) {
                middlePark = false;
            }
            telemetry.addData("Ideal Park?", middlePark);

            telemetry.update();
        }

        waitForStart();
        //robot.v4barPickup();

        switch ("none") {
            case "left":
                robot.drive.followTrajectorySequence(middle);
                break;
            case "right":
                robot.drive.followTrajectorySequence(left);
                break;
            case "none":
                robot.drive.followTrajectorySequence(right);
                break;
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
                    .lineToLinearHeading(new Pose2d(46, 55, Math.toRadians(-125))) // Corner Park
                    .build();
            robot.drive.followTrajectorySequence(cornerPark);
        }



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
//        PoseStorage.currentPose = robot.drive.getPoseEstimate();


    }
}