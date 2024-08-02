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
    private static double posTolerance = 0.5; //recommended constant
    private static double headingTolerance = 1; //recommended constant
    private static double centriDamp = 10; //centrifugal dampener(more amplifier)
    private static double decceleration = 97.42154202846739; // in inches/second this is for the glide braking
    private static PID translation = new PID(0.8, 0, 0.3, 0.3); //TUNABLE cmon bruhhh
    private static PID heading = new PID(0.3, 0, 0.3, 0.175); //TUNABLE its pid duhhh
    private static double radiusMulti = 0.008; //keep this a constant
    private static double piDAMP = 9; //how much to damp the goTO pid TUNABLE
    private static double minAccelNorm = 15; //minimum norm between centrifugal correction and velocity TUNABLE
    private static double minVel = 20; //minimum robot velocity for centrifugal correction to be used TUNABLE
    private static double strafeCentrifuge = 6; //how much more weight on centrifuge for when the robot's angle is skewed TUNABLE
    private static double lookAheadGain = 0.02; //how much to look ahead TUNABLE (25 is recommended follow distance at full speed)
    private static CurvePoint prevPoint = null;
    private static ArrayList<CurvePoint> path;
    public static boolean isBusy = false;
    public static Pose2d target;
    private static double movementSpeed = 0.0; // don't worry about this
    private static double turnSpeed = 0.0; // don't worry about this

    public static void setTarget(Pose2d input) {
        target = input;
    }

    public static void followCurve(ArrayList<CurvePoint> allPoints, MecanumDrive drive) {
        // bruhbruhbruhbruhbruh
        target = null; path = null;
        path = extendPoint(allPoints, drive);
        target = new Pose2d(allPoints.get(allPoints.size()-2).x, allPoints.get(allPoints.size()-2).y, allPoints.get(allPoints.size()-2).h);
    }

    public static ArrayList<CurvePoint> extendPoint(ArrayList<CurvePoint> allPoints, MecanumDrive drive) {
        //extend the path so it doesn't oscillate at the end
        //this is kind of hard to explain in comments the only real way to explain this is to draw it out
        double angle = Math.atan2(allPoints.get(allPoints.size()-1).y-drive.getPoseEstimate().getY(), allPoints.get(allPoints.size()-1).y-drive.getPoseEstimate().getX());
        if (allPoints.size()>1) {
            angle = Math.atan2(allPoints.get(allPoints.size()-1).y-allPoints.get(allPoints.size()-2).y, allPoints.get(allPoints.size()-1).x-allPoints.get(allPoints.size()-2).x);
        }
        Vector2d resultant = new Vector2d((allPoints.get(0).followDistance+6)*Math.cos(angle), (allPoints.get(0).followDistance+6)*Math.sin(angle));
        CurvePoint extend =new CurvePoint(allPoints.get(allPoints.size()-1).x+resultant.getX(), allPoints.get(allPoints.size()-1).y+resultant.getY(), allPoints.get(allPoints.size()-1).h, allPoints.get(allPoints.size()-1).moveSpeed, allPoints.get(allPoints.size()-1).turnSpeed, allPoints.get(allPoints.size()-1).followDistance, allPoints.get(allPoints.size()-1).slowDownTurnRadians, allPoints.get(allPoints.size()-1).slowDownTurnAmount);
        allPoints.add(extend);
        return allPoints;
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, double followRadius, MecanumDrive drive) {
        //find all intersections between the circle and piecewise path
        double vel = drive.getVelocity().norm();
        followRadius=followRadius*vel*lookAheadGain;
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        if (drive.getPoseEstimate().minus(target).vec().norm()<followMe.followDistance) {
            return new CurvePoint(target.getX(), target.getY(), target.getHeading(), followMe.moveSpeed,followMe.turnSpeed,followMe.followDistance, followMe.slowDownTurnRadians, followMe.slowDownTurnAmount);
        }
        for (int i = 0; i < pathPoints.size()-1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Vector2d> intersections = MathFunctions.lineCircleIntersection(drive.getPoseEstimate().vec(), followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 100000000;

            for (Vector2d thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.getY() - drive.getPoseEstimate().getY(), thisIntersection.getX() - drive.getPoseEstimate().getX());
                double deltaAngle;
                deltaAngle = Math.abs(MathFunctions.AngleDiff(angle, drive.getVelocity().angle()));

                if (deltaAngle < closestAngle) {
                    followMe = new CurvePoint(pathPoints.get(i + 1));
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        prevPoint=followMe;
        return followMe;
    }

    public static void goTO(MecanumDrive drive, Pose2d desired,double moveSpeed, double turSpeed) {
        setTarget(desired);
        movementSpeed=moveSpeed;
        turnSpeed = turSpeed;
    }

    public static void goToPosition(MecanumDrive drive, Pose2d position, Pose2d desired,double movementSpeed, double turnSpeed) {
        double x = desired.getX(); double y = desired.getY(); double preferredAngle = desired.getHeading();
        //use the centrifuge to calculate the force needed to keep the robot on the path
        Vector2d vel = drive.getVelocity();
        Vector2d accel1 = desired.minus(position).vec();
        Log.d("vel: ", vel.toString());
        Vector2d accel = scale(accel1, vel.norm());
        accel = accel.minus(vel);
        double centrifuge = Math.abs(antiRadius(vel, position, desired)*centriDamp);
        centrifuge = centrifuge+Math.abs(centrifuge*Math.sin(MathFunctions.AngleDiff(accel1.angle(), position.getHeading())))*strafeCentrifuge; // this is meant to double the correction when the robot's strafing because the velocity is obviously halfed
        if ((Math.abs(accel.norm())<minAccelNorm || position.minus(target).vec().norm()<minVel)) {
            centrifuge=0;
        } else {
            centrifuge=Math.abs(centrifuge);
        }
        Vector2d centrifuged;
        double ang = 0;
        if (MathFunctions.AngleWrap(accel.angle()-accel1.angle())<0) {
            ang = accel1.angle()-Math.PI/2;
        } else {
            ang = accel1.angle()+Math.PI/2;
        }
        ang=MathFunctions.AngleWrap(ang);
        preferredAngle=MathFunctions.AngleWrap(preferredAngle);
        centrifuged=new Vector2d(Math.cos(ang)*centrifuge, Math.sin(ang)*centrifuge);
        double distanceToTarget = Math.hypot(x - position.getX() + centrifuged.getX(), y - position.getY() + centrifuged.getY()); // adding the centrifuge vector and the target vector
        double absoluteAngleToTarget = Math.atan2(y - position.getY() + centrifuged.getY(), x - position.getX() + centrifuged.getX());
        double relativeAngleToPoint = MathFunctions.AngleDiff(absoluteAngleToTarget, position.getHeading());
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * (distanceToTarget);
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * (distanceToTarget);
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        if (relativeXToPoint == 0 && relativeYToPoint == 0) {
            movementXPower=0;
            movementYPower=0;
        }
        double pid;
        if (path!=null) pid = translation.update(distanceToTarget/path.get(0).followDistance);
        else pid = translation.update(distanceToTarget)/piDAMP;
        movementXPower = movementXPower * movementSpeed * (pid);
        movementYPower = movementYPower * movementSpeed * (pid);
        if (withinPos(position)) {
            movementXPower=0;
            movementYPower=0;
        }
        double headingPID = heading.update(MathFunctions.AngleDiff(preferredAngle, position.getHeading()));
        double movementTurn = -Range.clip(headingPID*(MathFunctions.AngleDiff(preferredAngle, position.getHeading()) > 0 ? 1 : -1), -1, 1)*turnSpeed;
        if (withinHead(position)) movementTurn = 0;
        double FL = (movementXPower - movementYPower + movementTurn);
        double BL = (movementXPower + movementYPower + movementTurn);
        double BR = (movementXPower - movementYPower - movementTurn);
        double FR = (movementXPower + movementYPower - movementTurn);
        drive.setMotorPowers(FL, BL, BR, FR);
    }

    public static Vector2d scale(Vector2d input, double scale) {
        //scale a vector to the prescribed magnitude
        double multi = scale/input.norm();
        return new Vector2d(input.getX()*multi, input.getY()*multi);
    }

    public static double antiRadius(Vector2d vel, Pose2d pose, Pose2d desired) {
        //anticentrifugal force correction
        Vector2d accel = desired.minus(pose).vec();
        accel = scale(accel, vel.norm());
        accel = accel.minus(vel);
        if ((accel.getY()==0)  && accel.getX()==0) return 0;
        double radius = Math.pow(1/Math.sin(desired.minus(pose).vec().angle()), 3)/(accel.getY()/accel.getX());
        double force = Math.pow(vel.norm(), 2)/radius*radiusMulti;
        return force;
    }

    public static boolean glide(MecanumDrive drive, Pose2d pose, Pose2d target) {
        //calculate the overshoot shit
        Vector2d vel = drive.getVelocity();
        double time = vel.norm()/decceleration;
        double distance = vel.norm()*time - 0.5*decceleration*Math.pow(time, 2);
        double angle = Math.atan2(vel.getY(), vel.getX());
        Vector2d estimated = new Vector2d(pose.getX()+distance*Math.cos(angle), pose.getY()+distance*Math.sin(angle));
        if (estimated.minus(pose.vec()).norm()<target.vec().minus(pose.vec()).norm()) return false;
//        Log.d("overshooted!", "overshooted!");
        Vector2d correction = target.vec().minus(estimated);
        //drive.setMotorPowers(0,0,0,0);
        goToPosition(drive, new Pose2d(0,0, pose.getHeading()), new Pose2d(correction.getX(), correction.getY(), target.getHeading()), 1, 1);
        return true;
    }

    public static boolean withinPos(Pose2d pose) {
        return pose.vec().minus(target.vec()).norm()<posTolerance;
    }

    public static boolean withinHead(Pose2d pose) {
        return Math.abs(MathFunctions.AngleDiff(target.getHeading(), pose.getHeading()))<Math.toRadians(headingTolerance);
    }

    public static void update(MecanumDrive drive) {
        drive.update();
        Pose2d pose = drive.getPoseEstimate();
        Log.d("robot pose: ", pose.toString());
        //if there is a path, but we are near the end kill the path
        if (target!=null && withinPos(pose) && withinHead(pose)) {
            target = null;
            path = null;
        }
        //if we have a path run it
        if (target!=null) {
//            Log.d("target: ", target.toString());
            if (glide(drive, pose, target)) {
            } else if (path!=null && !path.isEmpty() && (!withinPos(pose) || !withinHead(pose))) {
                CurvePoint followMe = getFollowPointPath(path, path.get(0).followDistance, drive); //Assume constant followDistance throughout path
                goToPosition(drive, drive.getPoseEstimate(), new Pose2d(followMe.x, followMe.y, followMe.h), followMe.moveSpeed, followMe.turnSpeed);
            } else {
                goToPosition(drive, drive.getPoseEstimate(), target, movementSpeed, turnSpeed);
            }
        } else {
            drive.setMotorPowers(0,0,0,0);
        }
    }

}