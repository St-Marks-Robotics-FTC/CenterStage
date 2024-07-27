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
    private Pose2d target = new Pose2d(30, 30, Math.toRadians(90));
    private boolean folow = true;
    private boolean foloow = true;
    private double loopTime = 0;
    public void runOpMode() {
        robot = new LebronClass(hardwareMap);
//        robot.drive.setPoseEstimate(new Pose2d(-40,-40,Math.toRadians(90)));
//        robot.drive.setPoseEstimate(new Pose2d(-50,-30,Math.toRadians(180)));
        robot.drive.setPoseEstimate(new Pose2d(0,0,0));
        waitForStart();
        //RobotMovement.goToPosition(robot.drive, robot.drive.getPoseEstimate(), target, 0.5, 0.5);
        //RobotMovement.setTarget(target);
        ArrayList<CurvePoint> path1 = new ArrayList<>();
//        path1.add(new CurvePoint(-40, 0, Math.toRadians(60), 0.75, 0.5, 25, 0, 0));
//        path1.add(new CurvePoint(0, 0, Math.toRadians(0), 0.75, 0.5, 25, 0, 0));
//        path1.add(new CurvePoint(0, -40, Math.toRadians(-60), 0.75, 0.5, 25, 0, 0));
//        path1.add(new CurvePoint(40, -40, Math.toRadians(0), 0.75, 0.5, 25, 0, 0));
//        path1.add(new CurvePoint(40, 0, Math.toRadians(90), 0.75, 0.5, 25, 0, 0));
//        path1.add(new CurvePoint(-50,-30,Math.toRadians(180), 1, 1, 20,0,0));
//        path1.add(new CurvePoint(-50,0,Math.toRadians(180), 1, 1, 20,0,0));
//        path1.add(new CurvePoint(50,0,Math.toRadians(0), 1, 1, 20,0,0));
//        path1.add(new CurvePoint(0,0,Math.toRadians(0), 0.8, 0.8, 20, 0, 0));
//        path1.add(new CurvePoint(0,-12,Math.toRadians(45), 0.8, 0.8, 5, 0, 0));
//        path1.add(new CurvePoint(24,-12,Math.toRadians(135), 0.8, 0.8, 5, 0, 0));
//        path1.add(new CurvePoint(24,12,Math.toRadians(-135), 0.8, 0.8, 5, 0, 0));
//        path1.add(new CurvePoint(0,12,Math.toRadians(-45), 0.8, 0.8, 20, 0, 0));
//        path1.add(new CurvePoint(0,0,Math.toRadians(0), 0.8, 0.8, 20, 0, 0));
//        RobotMovement.goTO(robot.drive, new Pose2d(-30, -20, Math.toRadians(-60)), 0.7, 0.75);

//        path1.add(new CurvePoint(40,0,Math.toRadians(-60), 1, 1, 30,0,0));
//        path1.add(new CurvePoint(40,-90,Math.toRadians(-90), 1, 1, 30,0,0));
//        path1.add(new CurvePoint(-8,-90,Math.toRadians(-180), 1, 1, 30,0,0));
//        path1.add(new CurvePoint(-8,-72,Math.toRadians(90), 1, 1, 30,0,0));
        path1.add(new CurvePoint(100,0,Math.toRadians(0), 1, 1, 20,0,0));

        ArrayList<CurvePoint> path2 = new ArrayList<>();
        path2.add(new CurvePoint(50,0,Math.toRadians(0), 1, 1, 20,0,0));
        path2.add(new CurvePoint(-50,0,Math.toRadians(0), 1, 1, 20,0,0));
        path2.add(new CurvePoint(-50,-30,Math.toRadians(180), 1, 1, 20,0,0));
        ElapsedTime time = new ElapsedTime();
        time.reset();
        int cycles = 0;
        while(!isStopRequested() && opModeIsActive()) {
//            RobotMovement.goTO(robot.drive, target, 1, 1);
            if (time.milliseconds()<7000 && folow) {
                RobotMovement.followCurve(path1, robot.drive);
                folow=false;
            } else if (time.milliseconds()>=7000 && time.milliseconds()<14000 && foloow) {
                Log.d("switched:", "switched");
//                RobotMovement.followCurve(path2, robot.drive);
                foloow=false;
            } else if (time.milliseconds()>=14000) {
//                folow = true;
//                foloow = true;
                time.reset();
                cycles++;
            }
//            if (cycles>3) break;
            RobotMovement.update(robot.drive);
            double loop = System.nanoTime();
            Log.d("hz ", Double.toString(1000000000 / (loop - loopTime)));
            loopTime = loop;
        }
    }
}
