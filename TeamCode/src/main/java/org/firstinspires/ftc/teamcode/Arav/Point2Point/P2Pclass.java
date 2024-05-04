package org.firstinspires.ftc.teamcode.Arav.Point2Point;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;

@Config
public class P2Pclass {
    public static double xP = 0.095;
    public static double xD = 0.011;

    public static double yP = 0.09;
    public static double yD = 0.011;

    public static double hP = 1.1;
    public static double hD = 0.045;

    public static double MAX_TRANSLATIONAL_SPEED = 0.5;
    public static double MAX_ROTATIONAL_SPEED = 0.4;
    public static double X_GAIN = 2.00;

    public static double ALLOWED_TRANSLATIONAL_ERROR = 1;
    public static double ALLOWED_HEADING_ERROR = 0.03;


    PIDFController xControl = new PIDFController(xP,0,xD,0);
    PIDFController yControl = new PIDFController(yP,0,yD,0);
    PIDFController thetaControl = new PIDFController(hP,0,hD,0);

    Pose2d target = new Pose2d(0,0,0);

    // Pass in target position and get Powers
    public double[] getPower(Pose2d target, Pose2d robotPosition){
        this.target = target;

        double xPower = xControl.calculate(robotPosition.getX(), target.getX());
        double yPower = yControl.calculate(robotPosition.getY(), target.getY());
        double hPower = thetaControl.calculate(robotPosition.getHeading(), target.getHeading());

        double x_rotated = xPower * Math.cos(robotPosition.getHeading()) - yPower * Math.sin(robotPosition.getHeading());
        double y_rotated = xPower * Math.sin(robotPosition.getHeading()) + yPower * Math.cos(robotPosition.getHeading());

        // x, y, theta input mixing
        double FL = x_rotated + y_rotated + hPower;
        double BL = x_rotated - y_rotated + hPower;
        double FR = x_rotated - y_rotated - hPower;
        double BR = x_rotated + y_rotated - hPower;

        return new double[]{FL, BL, FR, BR};
    }

    // Get target
    public Pose2d getTarget(){
        return target;
    }

    // Get Distance from Target
    public double getDistance(Pose2d robotPosition){
        return Math.hypot(robotPosition.getX() - target.getX(), robotPosition.getY() - target.getY());
    }




}
