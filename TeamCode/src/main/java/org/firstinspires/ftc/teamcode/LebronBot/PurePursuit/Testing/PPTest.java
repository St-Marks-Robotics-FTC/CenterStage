package org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.Testing;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;
import org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.RobotMovement;

@Autonomous
public class PPTest extends LinearOpMode {

    private LebronClass robot;
    private Pose2d target = new Pose2d(12, 0, Math.toRadians(0));
    public void runOpMode() {
        robot = new LebronClass(hardwareMap);
        robot.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        waitForStart();
        //RobotMovement.goToPosition(robot.drive, robot.drive.getPoseEstimate(), target, 0.5, 0.5);
        RobotMovement.goToPosition(robot.drive, robot.drive.getPoseEstimate(), target, 0.5, 0.5);
        while(!isStopRequested() && opModeIsActive()) {
            Log.d("robot pose: ", robot.drive.getPoseEstimate().toString());
            Log.d("distance: ", Double.toString(robot.drive.getPoseEstimate().vec().minus(target.vec()).norm()));
            RobotMovement.update(robot.drive);
//            if (robot.drive.getPoseEstimate().vec().minus(target.vec()).norm()>1) {
//                RobotMovement.goToPosition(robot.drive, robot.drive.getPoseEstimate(), target, 0.5, 0.5);
//            } else {
//                robot.drive.setMotorPowers(0, 0, 0, 0);
//            }
        }
    }
}