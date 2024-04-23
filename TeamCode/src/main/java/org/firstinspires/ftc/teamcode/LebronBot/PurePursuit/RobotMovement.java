package org.firstinspires.ftc.teamcode.LebronBot.PurePursuit;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.MecanumDrive;

import java.util.ArrayList;

public class RobotMovement {
    private static double turnTolerance=10;
    private static double posTolerance = 5;
    private static double turnP=0.5;
    public static boolean isBusy = false;
    public static CurvePoint target;

    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle, Pose2d robotPose, MecanumDrive drive) {
        CurvePoint followMe = getFollowPointPath(allPoints, allPoints.get(0).followDistance,robotPose);
        target = followMe;
        isBusy=true;
        goToPosition(drive, robotPose, new Pose2d(followMe.x, followMe.y, followAngle), followMe.moveSpeed, followMe.turnSpeed);
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, double followRadius, Pose2d robotPose) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        for (int i = 0; i < pathPoints.size() - 1; i++) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Vector2d> intersections = MathFunctions.lineCircleIntersection(robotPose.vec(), followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 100000000;

            for (Vector2d thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.getY() - robotPose.getY(), thisIntersection.getX() - robotPose.getX());
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - robotPose.getHeading()));

                if (deltaAngle < closestAngle) {
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
        double relativeAngleToPoint = MathFunctions.AngleWrap(absoluteAngleToTarget - (position.getHeading()));
        Log.d("relative angle: ", relativeAngleToPoint+" "+Math.toDegrees(relativeAngleToPoint));
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movementXPower = movementXPower * movementSpeed;
        movementYPower = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint + preferredAngle;

        double movementTurn = Range.clip(relativeTurnAngle*turnP, -1, 1)*turnSpeed;

        if (distanceToTarget < turnTolerance) {
            movementTurn = 0;
        }
        Log.d("movement: ", movementXPower+" "+movementYPower+" " + movementTurn);
        double LF = movementYPower + movementXPower + movementTurn;
        double LR = (movementYPower - movementXPower + movementTurn);
        double BR = movementYPower - movementXPower - movementTurn;
        double BF = (movementYPower + movementXPower - movementTurn);
        Log.d("movement powers: ", LF + " " + LR + " " + BR + " " + BF);
        drive.setMotorPowers(LF, LR, BR, BF);
    }

    public static double antiRadius(MecanumDrive drive) {
        //wolfpack's anticentrifugal force correction
        Vector2d vel = drive.getVelocity();
        Vector2d accel = drive.getAccel();
        double radius = (-Math.pow(1/Math.sin(Math.atan2(vel.getY(), vel.getX())), 3))/(accel.norm());
        return radius;
    }

    public static void update(MecanumDrive drive) {
        if (drive.getPoseEstimate().vec().minus(target.toPoint()).norm()<posTolerance) {
            isBusy=false;
            drive.setMotorPowers(0, 0, 0, 0);
            drive.update();
        }
    }

}
