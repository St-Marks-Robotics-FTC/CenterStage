package org.firstinspires.ftc.teamcode.LebronBot.PurePursuit;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.Vector;

@Config
public class RobotMovement {
    private static double posTolerance = 0.5;
    private static double headingTolerance = 1;
    public static boolean isBusy = false;
    public static Pose2d target;
    private static double dampener = 1;
    private static double decceleration = 135.337; // in inches/second this is for the wolfpack glide
    private static ArrayList<CurvePoint> path = new ArrayList<>();
    private static PID translation = new PID(0.015, 0, 0.5, 0.25);
    private static PID heading = new PID(0.4, 0, 0.2, 0.1);
    private static double radiusMulti = 0.008;
    private static CurvePoint prevPoint = null;

    public static void setTarget(Pose2d input) {
        target = input;
    }

    public static void followCurve(ArrayList<CurvePoint> allPoints, MecanumDrive drive) {
        // always keep prefAngle 90 degrees
        path = extendPoint(allPoints, drive);
        target = new Pose2d(allPoints.get(allPoints.size()-2).x, allPoints.get(allPoints.size()-2).y, allPoints.get(allPoints.size()-2).h);
    }

    public static ArrayList<CurvePoint> extendPoint(ArrayList<CurvePoint> allPoints, MecanumDrive drive) {
        double angle = Math.atan2(allPoints.get(allPoints.size()-1).y-drive.getPoseEstimate().getY(), allPoints.get(allPoints.size()-1).y-drive.getPoseEstimate().getX());
        if (allPoints.size()>1) {
            angle = Math.atan2(allPoints.get(allPoints.size()-1).y-allPoints.get(allPoints.size()-2).y, allPoints.get(allPoints.size()-1).x-allPoints.get(allPoints.size()-2).x);
        }
        Vector2d resultant = new Vector2d((allPoints.get(0).followDistance+6)*Math.cos(angle), (allPoints.get(0).followDistance+6)*Math.sin(angle));
        CurvePoint extend =new CurvePoint(allPoints.get(allPoints.size()-1).x+resultant.getX(), allPoints.get(allPoints.size()-1).y+resultant.getY(), allPoints.get(allPoints.size()-1).h, allPoints.get(allPoints.size()-1).moveSpeed, allPoints.get(allPoints.size()-1).turnSpeed, allPoints.get(allPoints.size()-1).followDistance, allPoints.get(allPoints.size()-1).slowDownTurnRadians, allPoints.get(allPoints.size()-1).slowDownTurnAmount);
        allPoints.add(extend);
//        for (CurvePoint p : allPoints) {
//            Log.d("point: ", Double.toString(p.x)+" "+Double.toString(p.y));
//        }
        return allPoints;
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, double followRadius, MecanumDrive drive) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        for (int i = 0; i < pathPoints.size()-1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Vector2d> intersections = MathFunctions.lineCircleIntersection(drive.getPoseEstimate().vec(), followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 100000000;

            for (Vector2d thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.getY() - drive.getPoseEstimate().getY(), thisIntersection.getX() - drive.getPoseEstimate().getX());
                double deltaAngle;
//                if (prevPoint!=null) {
//                    double prevAngle = prevPoint.toPoint().minus(drive.getPoseEstimate().vec()).angle();
//                    deltaAngle=Math.abs(MathFunctions.AngleDiff(angle,prevAngle));
//                } else {
//                    deltaAngle = Math.abs(MathFunctions.AngleDiff(angle, drive.getVelocity().angle()));
//                }
                deltaAngle = Math.abs(MathFunctions.AngleDiff(angle, drive.getVelocity().angle()));

                if (deltaAngle < closestAngle) {
                    followMe = new CurvePoint(pathPoints.get(i+1));
//                    Log.d("target heading: ", Double.toString(followMe.h));
//                    Log.d("i+1: ", Integer.toString(i+1));
//                    Log.d("bruhhhh: ", Double.toString(pathPoints.get(i+1).h));
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                    //setTarget(new Pose2d(pathPoints.get(i).toPoint().getX(),pathPoints.get(i).toPoint().getY(),pathPoints.get(i).h));
                }
            }
        }
        prevPoint=followMe;
        return followMe;
    }

    public static Vector2d speedLimit(MecanumDrive drive) {
        double ratio = 0.08;
        double minimum = 30;
        Vector2d output = new Vector2d((drive.getVelocity().getX()-minimum)*ratio, (drive.getVelocity().getY()-minimum)*ratio);
        return output;
    }

    public static void goToPosition(MecanumDrive drive, Pose2d position, Pose2d desired,double movementSpeed, double turnSpeed) {
//        setTarget(desired);
        double x = desired.getX(); double y = desired.getY(); double preferredAngle = desired.getHeading();
        double centrifuge = antiRadius(drive);
        Log.d("curvature: ", Double.toString(centrifuge));
//        position = new Pose2d(position.getX(), position.getY(), MathFunctions.AngleWrap(position.getHeading()));
        double distanceToTarget = Math.hypot(x - position.getX(), y - position.getY());
        double absoluteAngleToTarget = Math.atan2(y - position.getY(), x - position.getX());
//        Log.d("absoluteAngleToTarget: ", Double.toString(absoluteAngleToTarget));
//        double relativeAngleToPoint = MathFunctions.AngleWrap(absoluteAngleToTarget - (position.getHeading()));
        double relativeAngleToPoint = MathFunctions.AngleDiff(absoluteAngleToTarget, position.getHeading());
//        Log.d("relativeAngleToPoint", Double.toString(relativeAngleToPoint));
//        Log.d("prefferedAngle ", Double.toString(preferredAngle));
        if (centrifuge<2.5 && Math.abs(Math.toDegrees(drive.getAccel().angle()))>1) {
            centrifuge=0;
        } else {
            centrifuge=Math.abs(centrifuge);
        }
        Vector2d centrifuged = new Vector2d();
        double ang = 0;
        if (MathFunctions.AngleWrap(drive.getAccel().angle())<0) {
            ang = drive.getVelocity().angle()-Math.PI/2;
        } else {
            ang = drive.getVelocity().angle()+Math.PI/2;
        }
        centrifuged=new Vector2d(Math.cos(ang)*centrifuge, Math.sin(ang)*centrifuge);
        Log.d("centrifuged: ", centrifuged.toString());
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * (distanceToTarget);
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * (distanceToTarget);
//        Log.d("relativeXToPoint:  ", Double.toString(relativeXToPoint));
//        Log.d("relativeYToPoint:  ", Double.toString(relativeYToPoint));
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        if (relativeXToPoint == 0 && relativeYToPoint == 0) {
            movementXPower=0;
            movementYPower=0;
        }
        double pid = translation.update(distanceToTarget);
        pid = 1; // for now no pid because doesn't seem necessary
        //Log.d("pid: ", Double.toString(pid));
        //Vector2d limiter = speedLimit(drive);
        movementXPower = movementXPower * movementSpeed * (pid);// - limiter.getX();
        movementYPower = movementYPower * movementSpeed * (pid);// - limiter.getY();
        if (withinPos(drive)) {
            movementXPower=0;
            movementYPower=0;
        }
        if (drive.getPoseEstimate().vec().minus(target.vec()).norm()<5) relativeAngleToPoint=0;

        //double relativeTurnAngle = MathFunctions.AngleWrap(relativeAngleToPoint + preferredAngle);
        double headingPID = heading.update(MathFunctions.AngleDiff(preferredAngle, position.getHeading()));
//        headingPID = 1;
        //Log.d("relativeTurnAngle: ", Double.toString(relativeAngleToPoint));
//        Log.d("angle diff: ", Double.toString(MathFunctions.AngleDiff(preferredAngle, position.getHeading())));
        double movementTurn = -Range.clip(headingPID*(MathFunctions.AngleDiff(preferredAngle, position.getHeading()) > 0 ? 1 : -1), -1, 1)*turnSpeed;
        if (withinHead(drive)) movementTurn = 0;
        //Log.d("heading tolerance", Double.toString(Math.abs(MathFunctions.AngleDiff(target.getHeading(), drive.getPoseEstimate().getHeading()))));
//        Log.d("movementTurn: ", Double.toString(movementTurn));
        //if (distanceToTarget<4) movementTurn=0;
        double FL = (movementXPower - movementYPower + movementTurn);
        double BL = (movementXPower + movementYPower + movementTurn);
        double BR = (movementXPower - movementYPower - movementTurn);
        double FR = (movementXPower + movementYPower - movementTurn);
//        Log.d("pid", Double.toString(pid));
//        Log.d("heading pid", Double.toString(headingPID));
//        Log.d("movement: ", movementXPower+" "+movementYPower+" " + movementTurn);
//        Log.d("movement powers: ", FL + " " + BL + " " + BR + " " + FR);
//        Log.d("relative angle: ", relativeAngleToPoint+" "+Math.toDegrees(relativeAngleToPoint));
        drive.setMotorPowers(FL, BL, BR, FR);
    }

    public static double antiRadius(MecanumDrive drive) {
        //wolfpack's anticentrifugal force correction
        Vector2d vel = drive.getVelocity();
        Log.d("vel: ", vel.toString());
        Vector2d accel = drive.getAccel();
        Log.d("accel: ", accel.toString());
        if ((accel.getY()==0)  && accel.getX()==0) return 0;
        double radius = Math.pow(1/Math.sin(Math.atan2(vel.getY(),vel.getX())), 3)/(accel.getY()/accel.getX());
        Log.d("radius: ", Double.toString(radius));
        double force = Math.pow(vel.norm(), 2)/radius*radiusMulti;
        return force;
    }

    public static boolean glide(MecanumDrive drive) {
        Pose2d pose = drive.getPoseEstimate();
        Vector2d vel = drive.getVelocity();
        double time = vel.norm()/decceleration;
        double distance = vel.norm()*time - 0.5*decceleration*Math.pow(time, 2);
        double angle = Math.atan2(vel.getY(), vel.getX());
        Vector2d estimated = new Vector2d(pose.getX()+distance*Math.cos(angle), pose.getY()+distance*Math.sin(angle));
//        Log.d("estimate overhsoot: ", estimated.toString());
//        Log.d("first term: ", Double.toString(estimated.minus(pose.vec()).norm()));
//        Log.d("target vec: ", target.vec().toString());
//        Log.d("target norm: ", Double.toString(target.vec().minus(pose.vec()).norm()));
//        Log.d("second term: ", Double.toString(target.vec().minus(pose.vec()).norm()-2));
        if (estimated.minus(pose.vec()).norm()<target.vec().minus(pose.vec()).norm()-2) return false;
        Log.d("overshooted!", "overshooted!");
        Vector2d correction = target.vec().minus(estimated);
        //drive.setMotorPowers(0,0,0,0);
        goToPosition(drive, new Pose2d(0,0,drive.getPoseEstimate().getHeading()), new Pose2d(correction.getX(), correction.getY(), target.getHeading()), 1, 1);
//        if (withinPos(drive)) return true;
//        drive.followTrajectorySequenceAsync(drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
//                .lineToLinearHeading(new Pose2d(correction.getX(), correction.getY(), target.getHeading()))
//                .build());
        return true;
    }

    public static boolean withinPos(MecanumDrive drive) {
        return drive.getPoseEstimate().vec().minus(target.vec()).norm()<posTolerance;
    }

    public static boolean withinHead(MecanumDrive drive) {
        return Math.abs(MathFunctions.AngleDiff(target.getHeading(), drive.getPoseEstimate().getHeading()))<Math.toRadians(headingTolerance);
    }

    public static void update(MecanumDrive drive) {
        drive.update();
        Log.d("robot pose: ", drive.getPoseEstimate().toString());
        //Log.d("target: ", target.toString());
        //Log.d("bruh: ", Double.toString(Math.abs(MathFunctions.AngleWrap(drive.getPoseEstimate().getHeading()) - MathFunctions.AngleWrap(target.getHeading()))));
        if (withinPos(drive) && withinHead(drive)) target = null;
//        if (withinPos(drive)) {
//            drive.setWeightedDrivePower(new Pose2d(0,0,0));
//            goToPosition(drive, drive.getPoseEstimate(), target, 0, 1);
//            return;
//        }
        if (target!=null) {
            if (glide(drive)) {
            } else if (!path.isEmpty() && (!withinPos(drive) || !withinHead(drive))) {
                CurvePoint followMe = getFollowPointPath(path, path.get(0).followDistance,drive);
//                Log.d("target go to: ", new Pose2d(followMe.x, followMe.y, followMe.h).toString());
//                Log.d("yes!!!: ", path.get(path.size()-2).toPoint().toString());
                goToPosition(drive, drive.getPoseEstimate(), new Pose2d(followMe.x, followMe.y, followMe.h), followMe.moveSpeed, followMe.turnSpeed);
            }
        } else {
            drive.setMotorPowers(0,0,0,0);

            //Log.d("distance: ", Double.toString(drive.getPoseEstimate().vec().minus(target.vec()).norm()));
            //RobotMovement.goToPosition(drive, drive.getPoseEstimate(), target, 0.8, 0.8);
        }
    }

}