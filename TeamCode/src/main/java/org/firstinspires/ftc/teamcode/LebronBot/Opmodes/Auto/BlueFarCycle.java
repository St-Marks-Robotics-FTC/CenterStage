package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.Auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.Prop.BlueFarPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Disabled
@Autonomous (group = "Blue", preselectTeleOp = "JankTele")
public class BlueFarCycle extends LinearOpMode {
    FtcDashboard dashboard;

    public static String loc = "left";
    public static boolean middlePark = true;
    LebronClass robot;

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


        robot = new LebronClass(hardwareMap);

        Pose2d startPose = new Pose2d(-41, 63.5, Math.toRadians(90));
        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(startPose) // Truss side / No Prop Seen
                // Drive to spike
                .setReversed(true)
                .setTangent(Math.toRadians(-120))
                .splineToLinearHeading(new Pose2d(-33, 38, Math.toRadians(-30)), Math.toRadians(0))

//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.openLeftClaw();
//                })
                .waitSeconds(1) // Score Purple Spike

                //Intake a White
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-63, 11, Math.toRadians(180)), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    robot.closeLeftClaw();
//                })

                .waitSeconds(1)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-26, 8, Math.toRadians(180)), Math.toRadians(0))

                .splineToConstantHeading(new Vector2d(10, 8) , Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(43, 38),
                        Math.toRadians(0),
                        MecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(60))

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
                .splineToSplineHeading(new Pose2d(-39, 16, Math.toRadians(90)),
                        Math.toRadians(-90),
                        MecanumDrive.getVelocityConstraint(75, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(55))

//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.openLeftClaw();
//                })
                .waitSeconds(1) // Score Purple Spike

                //Intake a White
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-63, 10, Math.toRadians(180)),
                        Math.toRadians(180),
                        MecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL+Math.toRadians(35), DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(55))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    robot.closeLeftClaw();
//                })

                .waitSeconds(1)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-26, 8, Math.toRadians(180)), Math.toRadians(0))

                .splineToSplineHeading(new Pose2d(6, 8, Math.toRadians(180)),
                        Math.toRadians(0),
                        MecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(60))
                .splineToSplineHeading(new Pose2d(39, 33, Math.toRadians(180)),
                        Math.toRadians(45),
                        MecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL-Math.toRadians(0), DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(55))

//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    robot.v4barScore();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.openClaw();
//                })
                .waitSeconds(1) // Place Yellow



                .build();
        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startPose) // left
                .setTangent(Math.toRadians(-120))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-49.5, 23, Math.toRadians(60)),
                        Math.toRadians(-120),
                        MecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(55))

//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    robot.openLeftClaw();
//                })
                .waitSeconds(0.5) // Score Purple Spike

                //Intake a White
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-65, 11, Math.toRadians(180)), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    robot.closeLeftClaw();
//                })

                .waitSeconds(0.5)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(10, 8,Math.toRadians(180)),
                        Math.toRadians(0),
                        MecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(60))
                .splineToSplineHeading(new Pose2d(41, 27, Math.toRadians(180)),
                        Math.toRadians(45),
                        MecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(55))


//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    robot.v4barScore();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.openClaw();
//                })
                .waitSeconds(1) // Place Yellow
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
                //camera broken hard set the positions for now
                //robot.drive.followTrajectorySequence(right);
                //depositY=33;
                robot.drive.followTrajectorySequence(middle);
                depositY=28;
                break;
            case "right":
                robot.drive.followTrajectorySequence(middle);
                depositY=28;
                break;
            case "none":
                robot.drive.followTrajectorySequence(left);
                depositY=28;
                break;
        }
        int cycles = 0;
        while (time.milliseconds()<25000 && cycles<2) {
            cycles++;
            TrajectorySequence cycle = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .setReversed(true)
                    .setTangent(Math.toRadians(-120))
                    .splineToSplineHeading(new Pose2d(15, 8,Math.toRadians(180)), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-62.5, 11, Math.toRadians(180)),
                            Math.toRadians(180),
                            MecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            MecanumDrive.getAccelerationConstraint(55))
                    .waitSeconds(0.5)
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(6, 8,Math.toRadians(180)),
                            Math.toRadians(0),
                            MecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            MecanumDrive.getAccelerationConstraint(55))
                    .splineToSplineHeading(new Pose2d(39, depositY,Math.toRadians(180)),
                            Math.toRadians(45),
                            MecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            MecanumDrive.getAccelerationConstraint(60))
                    .resetConstraints()
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