package org.firstinspires.ftc.teamcode.jankbot.competition.auto;

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
import org.firstinspires.ftc.teamcode.Vision.Prop.BluePropThreshold;
import org.firstinspires.ftc.teamcode.jankbot.Jankbot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous (group = "Blue", preselectTeleOp = "JankTele")
public class BlueCycle extends LinearOpMode {
    FtcDashboard dashboard;

    public static String loc = "left";
    //MecanumDrive drive;
    Jankbot robot;

    private VisionPortal portal;
    private BluePropThreshold bluePropThreshold;
    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bluePropThreshold = new BluePropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(bluePropThreshold)
                .build();



        //drive = new MecanumDrive(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(12, -60, Math.toRadians(90)));
        robot = new Jankbot(hardwareMap);

        Pose2d startPose = new Pose2d(-41, 60, Math.toRadians(-90));
        robot.drive.setPoseEstimate(startPose);

        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-50, 20, Math.toRadians(-135)), Math.toRadians(-110))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {robot.special.releasePixel();})
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {robot.intake.tiltStack();})
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.intake.setIntake(0.8);})
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-61, 12, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.intake.setIntake(0);})
                //.UNSTABLE_addTemporalMarkerOffset(0, ()->{robot.transfer();})
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(20, 12) , Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()->{robot.outtake.v4barScore();})
                .splineToConstantHeading(new Vector2d(46, 31), Math.toRadians(45))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->robot.outtake.openBothClaw())
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->robot.outtake.closeBothClaw())
                .UNSTABLE_addTemporalMarkerOffset(0, ()->robot.outtake.v4barStow())
                .build();

        TrajectorySequence cycle = robot.drive.trajectorySequenceBuilder(startPose)
                .build();
    }
}
