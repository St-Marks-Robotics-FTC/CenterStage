package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.Testing;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.AprilTagRelocalize;

//@Disabled

@Config
@TeleOp (group = "test")
public class AprilTagTest extends OpMode {

    private LebronClass robot;
    private AprilTagRelocalize relocalize;

    private GamepadEx pad1;
    private final double STRAFE_SENSITIVITY = 1.0;
    private final double TURN_SENSITIVITY = 0.6;
    private int tag = 3;
    private int exposure = 6;
    private int gain = 100;
    //TODO: find the tag poses
    private Pose2d tagPose1 = new Pose2d(63, 41.5, Math.toRadians(180));
    private Pose2d tagPose2 = new Pose2d(63, 36, Math.toRadians(180));
    //1 = blue left
    //2 = blue middle
    //3 = blue right
    //4 = red left
    //5 = red middle
    //6 = red right
    @Override
    public void init() {
        robot = new LebronClass(hardwareMap);
        relocalize = new AprilTagRelocalize(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        robot.drive.setPoseEstimate(startPose);
        pad1 = new GamepadEx(gamepad1);
    }

    @Override
    public void init_loop() {
        Log.d("bruh: ", relocalize.getTagPos(new int[]{1,2,3,4,5,6}).toString());
        Pose2d relocalizePose = relocalize.getTagPos(new int[]{1,2,34,5,6});
        Pose2d predictPose = tagPose1.minus(relocalizePose);
        relocalizePose = new Pose2d(relocalizePose.getX(), relocalizePose.getY(), robot.drive.getPoseEstimate().getHeading());
        robot.drive.setPoseEstimate(relocalizePose);
        telemetry.addData("relocalizePose: ", relocalizePose.toString());
        telemetry.addData("estimated pose from ", predictPose.toString());
        telemetry.addData("Current Pose: ", robot.drive.getPoseEstimate().toString());
        telemetry.addData("Traveling to : ", relocalizePose.plus(robot.drive.getPoseEstimate()).toString());
        telemetry.addData("Exposure: ", exposure);
        telemetry.addData("Gain: ", gain);
        relocalize.setManualExposure(exposure, gain);
        if (gamepad1.a) {
            TrajectorySequence traj1 = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(36, -36, Math.toRadians(180)))
                    .build();
            robot.drive.followTrajectorySequenceAsync(traj1);
        } else {
            //normal drive control
            double r = Math.hypot(pad1.getLeftX(), pad1.getLeftY()); // Gets the amount that we want to translate
            double angleDC = Math.atan2(pad1.getLeftY(), pad1.getLeftX()) - Math.PI / 4;
            final double leftFrontSpeed = Range.clip((STRAFE_SENSITIVITY* r * Math.cos(angleDC) + TURN_SENSITIVITY * pad1.getRightX()), -1, 1);
            final double rightFrontSpeed = Range.clip((STRAFE_SENSITIVITY * r * Math.sin(angleDC) - TURN_SENSITIVITY * pad1.getRightX()), -1, 1);
            final double leftBackSpeed = Range.clip((STRAFE_SENSITIVITY * r * Math.sin(angleDC) + TURN_SENSITIVITY * pad1.getRightX()), -1, 1);
            final double rightBackSpeed = Range.clip((STRAFE_SENSITIVITY * r * Math.cos(angleDC) - TURN_SENSITIVITY * pad1.getRightX()), -1, 1);

            robot.drive.setMotorPowers(leftFrontSpeed, leftBackSpeed,  rightBackSpeed, rightFrontSpeed);
        }
//        if (gamepad1.a) exposure++;
//        if (gamepad1.b) exposure--;
//        if (gamepad1.x) gain+=10;
//        if (gamepad1.y) gain-=10;
        robot.drive.update();
    }

    @Override
    public void loop() {

    }
}
