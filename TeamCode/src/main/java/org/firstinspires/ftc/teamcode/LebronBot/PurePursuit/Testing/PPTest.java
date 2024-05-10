package org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.Testing;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;
import org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.RobotMovement;

import java.util.ArrayList;

@Autonomous
public class PPTest extends LinearOpMode {

    private LebronClass robot;
    private Pose2d target = new Pose2d(0, 0, Math.toRadians(90));
    public void runOpMode() {
        robot = new LebronClass(hardwareMap);
//        robot.drive.setPoseEstimate(new Pose2d(-40.5, -64, Math.toRadians(-90)));
        robot.drive.setPoseEstimate(new Pose2d(0,0,0));
        waitForStart();
        //RobotMovement.goToPosition(robot.drive, robot.drive.getPoseEstimate(), target, 0.5, 0.5);
        //RobotMovement.setTarget(target);
        ArrayList<CurvePoint> path = new ArrayList<>();
        path.add(new CurvePoint(100, 0, Math.toRadians(0), 1, 1, 24, 0, 0));
        path.add(new CurvePoint(100, -50, Math.toRadians(-90), 1, 1, 24, 0, 0));
//        path.add(new CurvePoint(-40.5, -10, Math.toRadians(-135), 1, 1, 24, 0, 0));
////        path.add(new CurvePoint(-60, -10, Math.toRadians(180), 1, 1, 24, 0, 0));
//        path.add(new CurvePoint(0, -10, Math.toRadians(180), 1, 1, 48, 0, 0));
//        path.add(new CurvePoint(40, -30, Math.toRadians(180), 1, 1, 48, 0, 0));
        RobotMovement.followCurve(path, robot.drive);
        while(!isStopRequested() && opModeIsActive()) {
//            RobotMovement.goToPosition(robot.drive, robot.drive.getPoseEstimate(), target, 1, 1);
            RobotMovement.update(robot.drive);
        }
    }
}
