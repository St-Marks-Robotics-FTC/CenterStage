package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.Prop.BlueFarPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Disabled
@Autonomous (group = "Blue", preselectTeleOp = "JankTele")
public class BlueFar2_0 extends LinearOpMode {
    FtcDashboard dashboard;

    public static String loc = "left";
    public static boolean middlePark = false;
    LebronClass robot;

    private VisionPortal portal;
    private BlueFarPropThreshold blueFarPropThreshold;
    private int delay = 10000;


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


        robot = new LebronClass(hardwareMap);

        Pose2d startPose = new Pose2d(-41, 63.5, Math.toRadians(90));
        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startPose) // Truss side / No Prop Seen
                // Drive to spike
                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-42, 26, Math.toRadians(135)), Math.toRadians(-110))

//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.openLeftClaw();
//                })
                .waitSeconds(1) // Score Purple Spike
                .back(4)
//
//                // Drive to Board
//                .setReversed(true)
//                .setTangent(Math.toRadians(-45))
//                .splineToLinearHeading(new Pose2d(-26, 12, Math.toRadians(180)), Math.toRadians(0))
//                .setTangent(0)
//                .splineToConstantHeading(new Vector2d(10, 12) , Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(48, 30), Math.toRadians(0))

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
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-37, 16, Math.toRadians(90)), Math.toRadians(-90))

//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.openLeftClaw();
//                })
                .waitSeconds(1) // Score Purple Spike
                .back(4)
//
//                .setReversed(true)
//                .setTangent(Math.toRadians(-60))
//                .splineToLinearHeading(new Pose2d(-38, 12, Math.toRadians(180)), Math.toRadians(0))
//                .setTangent(0)
//                .splineToConstantHeading(new Vector2d(10, 12) , Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(48, 30), Math.toRadians(0))

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
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-32, 33, Math.toRadians(0)), Math.toRadians(0))


//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.openLeftClaw();
//                })
                .waitSeconds(1) // Score Purple Spike
                .back(4)
//
//                .setReversed(true)
//                .setTangent(Math.toRadians(-135))
//                .splineToLinearHeading(new Pose2d(-38, 12, Math.toRadians(180)), Math.toRadians(0))
//                .setTangent(0)
//                .splineToConstantHeading(new Vector2d(10, 12) , Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(48, 41.5), Math.toRadians(0))


//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    robot.v4barScore();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.openClaw();
//                })
                .waitSeconds(2.5) // Place Yellow
                .build();





        // On Init
//        robot.closeClaw();

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
        sleep(delay);
//        robot.v4barPickup();
        switch (loc) {
            case "left":
                robot.drive.followTrajectorySequence(right);
                break;
            case "right":
                robot.drive.followTrajectorySequence(middle);
                break;
            case "none":
                robot.drive.followTrajectorySequence(left);
                break;
        }

//        if (middlePark) {
//            TrajectorySequence idealPark = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()) // Ideal Park pointed towards truss
//                    .setReversed(false)
//                    .lineToLinearHeading(new Pose2d(43, 13, Math.toRadians(-155)))
//                    .build();
//            robot.drive.followTrajectorySequence(idealPark);
//        } else {
//            TrajectorySequence cornerPark = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()) // Corner Park
//                    .setReversed(false)
//                    .lineToLinearHeading(new Pose2d(46, 58, Math.toRadians(-125))) // Corner Park
//                    .build();
//            robot.drive.followTrajectorySequence(cornerPark);
//        }



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
//        PoseStorage.currentPose = robot.drive.getPoseEstimate();


    }
}
