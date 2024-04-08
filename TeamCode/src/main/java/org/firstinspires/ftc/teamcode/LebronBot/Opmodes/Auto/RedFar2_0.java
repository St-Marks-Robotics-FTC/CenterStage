package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.Auto;

import android.util.Log;
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
import org.firstinspires.ftc.teamcode.Vision.AprilTag.AprilTagRelocalize;
import org.firstinspires.ftc.teamcode.Vision.Prop.RedFarPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
@Autonomous (group = "Blue", preselectTeleOp = "JankTele")
public class RedFar2_0 extends LinearOpMode {
    FtcDashboard dashboard;

    public static String loc = "left";
    public static boolean middlePark = false;
    LebronClass robot;

    private VisionPortal portal;
    private RedFarPropThreshold redFarPropThreshold;
    private AprilTagRelocalize relocalize;
    private int delay = 10000;
    private int exposure = 6;
    private int gain = 100;

    Pose2d poses[] = new Pose2d[]{new Pose2d(), new Pose2d(), new Pose2d(63, 41.5, Math.toRadians(180))};
    private int tagPose;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        redFarPropThreshold = new RedFarPropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size( 1280, 720))
                .addProcessor(redFarPropThreshold)
                .build();
        //portal.resumeStreaming();

        robot = new LebronClass(hardwareMap);

        Pose2d startPose = new Pose2d(-40.5, -64, Math.toRadians(-90));
        robot.drive.setPoseEstimate(startPose);
        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startPose) // Truss side / No Prop Seen
                // Drive to spike
                .setReversed(true)
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(-40, -38, Math.toRadians(-150)), Math.toRadians(60))
                //bring arm out
                //.waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.outtake.v4barPurple();
                    robot.intake.tiltStow();
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.1, () -> {
                    robot.outtake.v4BarAnglePurple();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.outtake.openRight();
                })
                .waitSeconds(0.8) // Score Purple Spike
                //stow back/prep for transfer
                .UNSTABLE_addTemporalMarkerOffset( 0, () -> {
                    robot.outtake.v4barStow();
                    robot.outtake.turretTransfer();
                })
                //go to stack
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-55, -13, Math.toRadians(180)), Math.toRadians(180))
                //.waitSeconds(5)
                //turn on intake
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntake(1);
                    robot.intake.tiltStack();
                    robot.outtake.setV4BarAngle(robot.outtake.angleTransfer+0.015);
                })
                .waitSeconds(0.5) //wait for robot to stabilize
                //line into the stack and pray intake works
                .lineToLinearHeading(new Pose2d(-61.5, -13, Math.toRadians(180)))
                //.waitSeconds(5)
                //start transfer sequence
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    robot.intake.setIntake(0);
                    robot.intake.tiltUp();
                })
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.outtake.v4barTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    robot.outtake.closeBothClaws();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.85, () -> {
                    robot.outtake.setV4Bar(robot.outtake.v4barStow);
                    robot.outtake.turretTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(2.25, () -> {
                    robot.intake.tiltStow();
                    robot.outtake.v4barAngleStow();
                    robot.outtake.turretTransfer();
                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    relocalize = new AprilTagRelocalize(hardwareMap);
//                    relocalize.setManualExposure(exposure, gain);
//                    relocalize.getTagPos(tagPose);
//                })
//                .waitSeconds(5)
                //drive to the backdrop
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-24, -10, Math.toRadians(180)), Math.toRadians(0))
                //prep the outtake
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.outtake.v4BarAuto();
                    robot.outtake.turretTo(1);
                })
                //finish going to backdrop
                .splineToSplineHeading(new Pose2d(5, -10, Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(45.5, -45, Math.toRadians(180)), Math.toRadians(-45))
//                //release
//                //.waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.outtake.openBothClaws();
//                    Pose2d relocalizePose = relocalize.getTagPos(tagPose);
//                    Log.d("Relocalize Pose: ", relocalizePose.toString());
//                    if (relocalizePose.getX()<=72) {
//                        robot.drive.setPoseEstimate(poses[tagPose-1].minus(relocalizePose));
//                    }
                })
                .waitSeconds(1.5)
                .build();

        TrajectorySequence middle = robot.drive.trajectorySequenceBuilder(startPose) // middle
                // Drive to spike
                .setReversed(true)
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(-48, -32, Math.toRadians(-160)), Math.toRadians(120))
                //bring arm out
                //.waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.outtake.v4barPurple();
                    robot.intake.tiltStow();
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.1, () -> {
                    robot.outtake.v4BarAnglePurple();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.outtake.openRight();
                })
                .waitSeconds(0.8) // Score Purple Spike
                //stow back/prep for transfer
                .UNSTABLE_addTemporalMarkerOffset( 0, () -> {
                    robot.outtake.v4barStow();
                    robot.outtake.turretTransfer();
                })
                //go to stack
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-55, -13, Math.toRadians(180)), Math.toRadians(180))
                //.waitSeconds(5)
                //turn on intake
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntake(1);
                    robot.intake.tiltStack();
                    robot.outtake.v4barAngleTransfer();
                })
                .waitSeconds(0.5) //wait for robot to stabilize
                //line into the stack and pray intake works
                .lineToLinearHeading(new Pose2d(-59.5, -13, Math.toRadians(180)))
                //.waitSeconds(5)
                //start transfer sequence
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    robot.intake.setIntake(0);
                    robot.intake.tiltUp();
                })
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.outtake.v4barTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    robot.outtake.closeBothClaws();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.85, () -> {
                    robot.outtake.setV4Bar(robot.outtake.v4barStow);
                    robot.outtake.turretTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(2.25, () -> {
                    robot.intake.tiltStow();
                    robot.outtake.v4barAngleStow();
                    robot.outtake.turretTransfer();
                })
                //.waitSeconds(5)
                //drive to the backdrop
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-24, -10), Math.toRadians(0))
                //prep the outtake
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.outtake.v4BarAuto();
                    robot.outtake.turretTo(1);
                })
                //finish going to backdrop
                .splineToConstantHeading(new Vector2d(5, -10), Math.toRadians(-10))
                .splineToConstantHeading(new Vector2d(46.5, -36), Math.toRadians(-40))
                //release
                //.waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
                    robot.outtake.openBothClaws();
                })
                .waitSeconds(1.5)
                .build();
        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(startPose) // left
                // Drive to spike
                .setReversed(true)
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(-56, -24, Math.toRadians(120)), Math.toRadians(120))
                //bring arm out
                //.waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.outtake.v4barPurple();
                    robot.intake.tiltStow();
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.1, () -> {
                    robot.outtake.v4BarAnglePurple();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.outtake.openRight();
                })
                .waitSeconds(0.8) // Score Purple Spike
                //stow back/prep for transfer
                .UNSTABLE_addTemporalMarkerOffset( 0, () -> {
                    robot.outtake.v4barStow();
                    robot.outtake.turretTransfer();
                })
                //go to stack
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-55, -13, Math.toRadians(180)), Math.toRadians(180))
                //.waitSeconds(5)
                //turn on intake
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntake(1);
                    robot.intake.tiltStack();
                    robot.outtake.v4barAngleTransfer();
                })
                .waitSeconds(0.5) //wait for robot to stabilize
                //line into the stack and pray intake works
                .lineToLinearHeading(new Pose2d(-60, -13, Math.toRadians(180)))
                //.waitSeconds(5)
                //start transfer sequence
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    robot.intake.setIntake(0);
                    robot.intake.tiltUp();
                    robot.outtake.v4barAngleTransfer();
                })
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    robot.outtake.v4barTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    robot.outtake.closeBothClaws();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.85, () -> {
                    robot.outtake.setV4Bar(robot.outtake.v4barStow);
                    robot.outtake.turretTransfer();
                })
                .UNSTABLE_addTemporalMarkerOffset(2.25, () -> {
                    robot.intake.tiltStow();
                    robot.outtake.v4barAngleStow();
                    robot.outtake.turretTransfer();
                })
                //.waitSeconds(5)
                //drive to the backdrop
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-24, -10), Math.toRadians(0))
                .addDisplacementMarker(() -> {

                })
                //prep the outtake
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.outtake.v4BarAuto();
                    robot.outtake.turretTo(1);
                })
                //finish going to backdrop
                .splineToConstantHeading(new Vector2d(5, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45.5, -32), Math.toRadians(-45))
                //release
                //.waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.outtake.openBothClaws();
                })
                .waitSeconds(1.25)
                .build();

        // On Init
//        robot.closeClaw();
        robot.outtake.closeRightMore();
        robot.outtake.turretTransfer();
        robot.intake.tiltUp();

        while (opModeInInit() && !isStopRequested()) {
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
        switch (redFarPropThreshold.getPropPosition()) {
            case "none":
                robot.drive.followTrajectorySequence(right);
                tagPose=3;
                break;
            case "right":
                robot.drive.followTrajectorySequence(middle);
                tagPose=2;
                break;
            case "left":
                robot.drive.followTrajectorySequence(left);
                tagPose=1;
                break;
        }

        TrajectorySequence path = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                //stow back
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.outtake.v4barStow();
//                    robot.outtake.turretTransfer();
//                })
                //return back to stack
                .setTangent(Math.toRadians(150))
                .splineToConstantHeading(new Vector2d(13, -15), Math.toRadians(150))
                .splineToConstantHeading(new Vector2d(-24, -15), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-55, -15), Math.toRadians(180))
                //turn on intake
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.intake.setIntake(1);
//                    robot.intake.tiltStackTo(1);
//                })
                .waitSeconds(0.5) //wait for robot to stabilize
                //line into the stack and pray intake works
                .lineToLinearHeading(new Pose2d(-60.5, -13, Math.toRadians(180)))
                //start transfer sequence
//                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//                    robot.intake.setIntake(-0.3);
//                    robot.intake.tiltUp();
//                    robot.outtake.v4barAngleTransfer();
//                    robot.outtake.openBothClaws();
//                })
                .waitSeconds(0.4)
//                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
//                    robot.intake.setIntake(0);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    robot.outtake.turretTransfer();
//                    robot.outtake.v4barTransfer();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.15, () -> {
//                    robot.outtake.closeBothClaws();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.75, () -> {
//                    robot.outtake.setV4Bar(robot.outtake.v4barStow);
//                    robot.outtake.turretTransfer();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(2.25, () -> {
//                    robot.intake.tiltStow();
//                    robot.outtake.v4barAngleStow();
//                    robot.outtake.turretTransfer();
//                })
                //drive to the backdrop
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-24, -10), Math.toRadians(0))
                //prep the outtake
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.outtake.v4BarAuto();
//                    robot.outtake.turretTo(1);
//                })
                //finish going to backdrop
                .splineToConstantHeading(new Vector2d(15, -10), Math.toRadians(-20))
                .splineToConstantHeading(new Vector2d(46, -35), Math.toRadians(-45))
                //release
//                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
//                    robot.outtake.openBothClaws();
//                })
                .waitSeconds(1.25)
                //stow back
//                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
//                    robot.outtake.v4barStow();
//                    robot.outtake.turretTransfer();
//                })
                //return back to stack
                .setTangent(Math.toRadians(150))
                .splineToConstantHeading(new Vector2d(13, -13), Math.toRadians(150))
                .splineToConstantHeading(new Vector2d(-24, -13), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-55, -13), Math.toRadians(180))
                //turn on intake
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.intake.setIntake(1);
//                    robot.intake.tiltDown();
//                })
                .waitSeconds(0.5) //wait for robot to stabilize
                //line into the stack and pray intake works
                .lineToLinearHeading(new Pose2d(-60.5, -13, Math.toRadians(180)))
                //start transfer sequence
//                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//                    robot.intake.setIntake(-0.3);
//                    robot.intake.tiltUp();
//                    robot.outtake.v4barAngleTransfer();
//                    robot.outtake.openBothClaws();
//                })
                .waitSeconds(0.4)
//                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
//                    robot.intake.setIntake(0);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    robot.outtake.turretTransfer();
//                    robot.outtake.v4barTransfer();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.15, () -> {
//                    robot.outtake.closeBothClaws();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.75, () -> {
//                    robot.outtake.setV4Bar(robot.outtake.v4barStow);
//                    robot.outtake.turretTransfer();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(2.25, () -> {
//                    robot.intake.tiltStow();
//                    robot.outtake.v4barAngleStow();
//                    robot.outtake.turretTransfer();
//                })
                //drive to the backdrop
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-24, -10), Math.toRadians(0))
                //prep the outtake
//                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
//                    robot.outtake.v4BarAuto();
//                    robot.outtake.turretTo(1);
//                })
                //finish going to backdrop
                .splineToConstantHeading(new Vector2d(15, -10), Math.toRadians(-20))
                .splineToConstantHeading(new Vector2d(48, -35), Math.toRadians(-45))
                //release
//                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
//                    robot.outtake.openBothClaws();
//                })
//                .waitSeconds(1.5)
//                //stow back
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.outtake.v4barStow();
//                    robot.outtake.turretTransfer();
//                })
                .build();
        //relocalize.visionPortal.close();
        //robot.drive.followTrajectorySequence(path);
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