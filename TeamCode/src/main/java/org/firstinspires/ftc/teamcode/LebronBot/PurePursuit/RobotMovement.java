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

@Config
public class RobotMovement {
    private static double turnTolerance=10;
    private static double posTolerance = 1;
    private static double headingTolerance = 1;
    private static double turnP=1;
    public static boolean isBusy = false;
    public static Pose2d target;
    public static Pose2d endPose;
    private static double dampener = 1;
    private static double decceleration = 0.5; // in inches/this is for the wolfpack glide
    private static ArrayList<CurvePoint> path = new ArrayList<>();
    private static double followAngle;
    private static PID translation = new PID(0.2, 0, 0.1);
    private static PID heading = new PID(1, 0, 0);

//    public static void glide(MecanumDrive drive, Pose2d position, Pose2d desired, double movementSpeed, double turnSpeed) {
//        target =
//    }

    public static void setTarget(Pose2d input) {
        target = input;
    }

    public static void followCurve(ArrayList<CurvePoint> allPoints) {
        // always keep prefAngle 90 degrees
        path = extendPoint(allPoints);
        target = new Pose2d(allPoints.get(allPoints.size()-2).x, allPoints.get(allPoints.size()-2).y, allPoints.get(allPoints.size()-2).h);
    }

    public static ArrayList<CurvePoint> extendPoint(ArrayList<CurvePoint> allPoints) {
        double angle = Math.atan2(allPoints.get(allPoints.size()-1).y-allPoints.get(allPoints.size()-2).y, allPoints.get(allPoints.size()-1).x-allPoints.get(allPoints.size()-2).x);
        Vector2d resultant = new Vector2d(allPoints.get(0).followDistance*Math.cos(angle), allPoints.get(0).followDistance*Math.sin(angle));
        CurvePoint extend =new CurvePoint(allPoints.get(allPoints.size()-1).x+resultant.getX(), allPoints.get(allPoints.size()-1).y+resultant.getY(), allPoints.get(allPoints.size()-1).h, allPoints.get(allPoints.size()-1).moveSpeed, allPoints.get(allPoints.size()-1).turnSpeed, allPoints.get(allPoints.size()-1).followDistance, allPoints.get(allPoints.size()-1).slowDownTurnRadians, allPoints.get(allPoints.size()-1).slowDownTurnAmount);
        allPoints.add(extend);
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
                double deltaAngle = Math.abs(MathFunctions.AngleDiff(angle, drive.getVelocity().angle()));

                if (deltaAngle < closestAngle) {
                    followMe = new CurvePoint(pathPoints.get(i+1));
                    Log.d("target heading: ", Double.toString(followMe.h));
                    Log.d("i+1: ", Integer.toString(i+1));
                    Log.d("bruhhhh: ", Double.toString(pathPoints.get(i+1).h));
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public static void goToPosition(MecanumDrive drive, Pose2d position, Pose2d desired,double movementSpeed, double turnSpeed) {
        double x = desired.getX(); double y = desired.getY(); double preferredAngle = desired.getHeading();
        double distanceToTarget = Math.hypot(x - position.getX(), y - position.getY());
        double absoluteAngleToTarget = Math.atan2(y - position.getY(), x - position.getX());
//        Log.d("absoluteAngleToTarget: ", Double.toString(absoluteAngleToTarget));
//        double relativeAngleToPoint = MathFunctions.AngleWrap(absoluteAngleToTarget - (position.getHeading()));
        double relativeAngleToPoint = MathFunctions.AngleDiff(absoluteAngleToTarget, position.getHeading());
//        Log.d("relativeAngleToPoint", Double.toString(relativeAngleToPoint));
//        Log.d("prefferedAngle ", Double.toString(preferredAngle));
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
//        Log.d("relativeXToPoint:  ", Double.toString(relativeXToPoint));
//        Log.d("relativeYToPoint:  ", Double.toString(relativeYToPoint));
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint)) / dampener;
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint)) / dampener;
        if (relativeXToPoint == 0 && relativeYToPoint == 0) {
            movementXPower=0;
            movementYPower=0;
        }
        double pid = translation.update(distanceToTarget);
        Log.d("pid: ", Double.toString(pid));
        movementXPower = movementXPower * movementSpeed * (pid);
        movementYPower = movementYPower * movementSpeed * (pid);
        if (withinPos(drive)) {
            movementXPower=0;
            movementYPower=0;
        }

        double relativeTurnAngle = relativeAngleToPoint + preferredAngle;
//        Log.d("relativeTurnAngle: ", Double.toString(relativeTurnAngle));
        double movementTurn = -Range.clip(MathFunctions.AngleDiff(preferredAngle, position.getHeading())*turnP, -1, 1)*turnSpeed;
//        Log.d("movementTurn: ", Double.toString(movementTurn));
        //if (distanceToTarget<4) movementTurn=0;
        double FL = (movementXPower - movementYPower + movementTurn);
        double BL = (movementXPower + movementYPower + movementTurn);
        double BR = (movementXPower - movementYPower - movementTurn);
        double FR = (movementXPower + movementYPower - movementTurn);
//        Log.d("movement: ", movementXPower+" "+movementYPower+" " + movementTurn);
//        Log.d("movement powers: ", FL + " " + BL + " " + BR + " " + FR);
//        Log.d("relative angle: ", relativeAngleToPoint+" "+Math.toDegrees(relativeAngleToPoint));
        drive.setMotorPowers(FL, BL, BR, FR);
    }

    public static double antiRadius(MecanumDrive drive) {
        //wolfpack's anticentrifugal force correction
        Vector2d vel = drive.getVelocity();
        Vector2d accel = drive.getAccel();
        if (((accel.getY()*vel.getX())-(accel.getX()*vel.getY())) == 0) return 0;
        double radius = Math.pow(vel.norm(), 2.5)/((accel.getY()*vel.getX())-(accel.getX()*vel.getY()));
        return radius;
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
        if (path!=null) {
            if (target!=null && withinPos(drive) && withinHead(drive)) {
                goToPosition(drive, drive.getPoseEstimate(), target,path.get(0).moveSpeed,path.get(0).turnSpeed);
            }
            isBusy=false;
            drive.setMotorPowers(0, 0, 0, 0);
            CurvePoint followMe = getFollowPointPath(path, path.get(0).followDistance,drive);
            Log.d("target go to: ", new Pose2d(followMe.x, followMe.y, followMe.h).toString());
            goToPosition(drive, drive.getPoseEstimate(), new Pose2d(followMe.x, followMe.y, followMe.h), followMe.moveSpeed, followMe.turnSpeed);
        } else {
            isBusy=true;
            path=null;
            //Log.d("distance: ", Double.toString(drive.getPoseEstimate().vec().minus(target.vec()).norm()));
            //RobotMovement.goToPosition(drive, drive.getPoseEstimate(), target, 0.8, 0.8);
        }
    }

}
