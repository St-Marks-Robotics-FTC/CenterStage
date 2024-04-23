package org.firstinspires.ftc.teamcode.LebronBot.PurePursuit;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class CurvePoint {
    double x;
    double y;
    double moveSpeed;
    double turnSpeed;
    double followDistance;
    double slowDownTurnRadians;
    double slowDownTurnAmount;

    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnRadians, double slowDownTurnAmount) {
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
    }

    public CurvePoint(CurvePoint thisPoint) {
        this.x = thisPoint.x;
        this.y = thisPoint.y;
        this.moveSpeed = thisPoint.moveSpeed;
        this.turnSpeed = thisPoint.turnSpeed;
        this.followDistance = thisPoint.followDistance;
        this.slowDownTurnRadians = thisPoint.slowDownTurnRadians;
        this.slowDownTurnAmount = thisPoint.slowDownTurnAmount;
    }

    public Vector2d toPoint() {
        return new Vector2d(x,y);
    }

    public void setPoint(Vector2d point) {
        x=point.getX();
        y=point.getY();
    }
}
