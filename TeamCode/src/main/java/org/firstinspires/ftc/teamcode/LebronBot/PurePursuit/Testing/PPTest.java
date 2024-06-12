package org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.Testing;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;
import org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.RobotMovement;

import java.util.ArrayList;

@Autonomous
public class PPTest extends LinearOpMode {

    private LebronClass robot;
    private Pose2d target = new Pose2d(0, 0, Math.toRadians(90));
    private boolean folow = true;
    private boolean foloow = true;
    public void runOpMode() {
        robot = new LebronClass(hardwareMap);
//        robot.drive.setPoseEstimate(new Pose2d(-40.5, -64, Math.toRadians(-90)));
//        robot.drive.setPoseEstimate(new Pose2d(-50,-50,Math.toRadians(90)));
        robot.drive.setPoseEstimate(new Pose2d(-50,0,0));
        waitForStart();
        //RobotMovement.goToPosition(robot.drive, robot.drive.getPoseEstimate(), target, 0.5, 0.5);
        //RobotMovement.setTarget(target);
        ArrayList<CurvePoint> path1 = new ArrayList<>();
        path1.add(new CurvePoint(-50, 0, Math.toRadians(0), 1, 1, 30, 0, 0));
//        path1.add(new CurvePoint(0, 0, Math.toRadians(0), 0.85, 0.5, 20, 0, 0));
//        path1.add(new CurvePoint(0, -50, Math.toRadians(-60), 0.85, 0.5, 20, 0, 0));
//        path1.add(new CurvePoint(50, -50, Math.toRadians(0), 0.85, 0.5, 20, 0, 0));
//        path1.add(new CurvePoint(50, 0, Math.toRadians(90), 0.85, 0.5, 20, 0, 0));
//        path1.add(new CurvePoint(-50,0,0, 1, 1, 30,0,0));
        path1.add(new CurvePoint(50,0,0, 1, 1, 30,0,0));
        path1.add(new CurvePoint(50,30,Math.toRadians(90), 1, 1, 30,0,0));
//        path1.add(new CurvePoint(-40.5, -34, Math.toRadians(-135), 0.75, 0.75, 24, 0, 0));
//        ArrayList<CurvePoint> path2 = new ArrayList<>();
//        path2.add(new CurvePoint(-60, -10, Math.toRadians(180), 1, 1, 24, 0, 0));
//        ArrayList<CurvePoint> path3 = new ArrayList<>();
//        path3.add(new CurvePoint(0, -10, Math.toRadians(180), 1, 1, 24, 0, 0));
//        path3.add(new CurvePoint(48, -36, Math.toRadians(180), 1, 1, 24, 0, 0));
        RobotMovement.followCurve(path1, robot.drive);
//        RobotMovement.goTO(robot.drive, new Pose2d(-30, -5, Math.toRadians(-60)), 0.6, 0.6);
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while(!isStopRequested() && opModeIsActive()) {
//            RobotMovement.goToPosition(robot.drive, robot.drive.getPoseEstimate(), target, 1, 1);
//            if (time.milliseconds()>10000 && foloow) {
//                RobotMovement.followCurve(path3, robot.drive);
//                foloow = false;
//            } else if (time.milliseconds()>5000 && folow){
//                RobotMovement.followCurve(path2, robot.drive);
//                folow = false;
//            }

            RobotMovement.update(robot.drive);
        }
    }
}
