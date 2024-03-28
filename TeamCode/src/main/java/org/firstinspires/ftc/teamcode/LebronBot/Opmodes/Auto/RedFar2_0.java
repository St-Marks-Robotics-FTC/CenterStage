package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.Auto;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.Prop.RedFarPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous (group = "Blue", preselectTeleOp = "JankTele")
public class RedFar2_0 extends LinearOpMode {
    FtcDashboard dashboard;

    public static String loc = "left";
    public static boolean middlePark = false;
    LebronClass robot;

    private VisionPortal portal;
    private RedFarPropThreshold redFarPropThreshold;
    private int delay = 10000;


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


        robot = new LebronClass(hardwareMap);

        Pose2d startPose = new Pose2d(-41, -62.5, Math.toRadians(-90));
        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startPose) // Truss side / No Prop Seen
                // Drive to spike
                .setReversed(true)
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(-37, -33, Math.toRadians(-150)), Math.toRadians(60))

//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.openLeftClaw();
//                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.outtake.v4BarAnglePurple();
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.1, () -> {
                    robot.outtake.v4barPurple();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.outtake.openLeft();
                })
                .waitSeconds(1) // Score Purple Spike
                .UNSTABLE_addTemporalMarkerOffset( 0, () -> {
                    robot.outtake.v4barStow();
                    robot.outtake.turretTransfer();
                })
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-55, -13, Math.toRadians(180)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntake(1);
                    robot.intake.tiltStack();
                })
                .waitSeconds(1)
//                .UNSTABLE_addTemporalMarkerOffset( 0, () -> {
//                    robot.drive.setMotorPowersAndTrack(0.2,0.2,0.2,0.2);
//                })
//                .UNSTABLE_addTemporalMarkerOffset( 1, () -> {
//                    robot.drive.setMotorPowers(0,0,0,0);
//                })
                .lineToLinearHeading(new Pose2d(-61, -13, Math.toRadians(180)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntake(0);
                    robot.intake.tiltUp();
                })
                .setTangent(Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.outtake.openBothClaws();
                    robot.outtake.turretTransfer();
                    robot.outtake.v4barTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    robot.outtake.closeBothClaws();
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                    robot.outtake.setV4Bar(robot.outtake.v4barStow);
                    robot.outtake.turretTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(3.25, () -> {
                    robot.intake.tiltStow();
                    robot.outtake.v4barAngleStow();
                    robot.outtake.turretTransfer();
                })
                .splineToConstantHeading(new Vector2d(-24, -10), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.outtake.v4barScore();
                    robot.outtake.turretTo(1);
                })
                .splineToConstantHeading(new Vector2d(15, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -42), Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.outtake.openBothClaws();
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.outtake.v4barStow();
                    robot.outtake.turretTransfer();
                })
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(13, -12), Math.toRadians(150))
                .splineToConstantHeading(new Vector2d(-24, -12), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-61, -12), Math.toRadians(180))
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
                //.waitSeconds(2.5) // Place Yellow

                .build();
        TrajectorySequence middle = robot.drive.trajectorySequenceBuilder(startPose) // middle
                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-37, -16, Math.toRadians(-90)), Math.toRadians(90))

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
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-32, -33, Math.toRadians(0)), Math.toRadians(0))


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
        robot.outtake.closeLeft();
        robot.outtake.turretTransfer();
        robot.intake.tiltUp();

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
        //sleep(delay);
//        robot.v4barPickup();
        switch ("left") {
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
//                    .lineToLinearHeading(new Pose2d(43, -13, Math.toRadians(155)))
//                    .build();
//            robot.drive.followTrajectorySequence(idealPark);
//        } else {
//            TrajectorySequence cornerPark = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate()) // Corner Park
//                    .setReversed(false)
//                    .lineToLinearHeading(new Pose2d(46, -58, Math.toRadians(125))) // Corner Park
//                    .build();
//            robot.drive.followTrajectorySequence(cornerPark);
//        }



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
//        PoseStorage.currentPose = robot.drive.getPoseEstimate();


    }
}