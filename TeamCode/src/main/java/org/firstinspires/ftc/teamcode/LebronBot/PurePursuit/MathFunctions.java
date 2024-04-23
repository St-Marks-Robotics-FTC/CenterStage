package org.firstinspires.ftc.teamcode.LebronBot.PurePursuit;




import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class MathFunctions {
    /**
     * Wraps an angle to the range -pi to pi radians.
     *
     * @param angle the angle to wrap
     * @return the wrapped angle
     */
    public static double AngleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;

    }

    public static ArrayList<Vector2d> lineCircleIntersection(Vector2d circleCenter, double radius, Vector2d lineStart, Vector2d lineEnd) {
        if (Math.abs(lineStart.getY() - lineEnd.getY()) < 0.003) {
            lineStart = new Vector2d(lineStart.getX(), lineEnd.getY()+0.0003);
        }
        if (Math.abs(lineStart.getX() - lineEnd.getX()) < 0.003) {
            lineStart = new Vector2d(lineEnd.getX()+0.0003, lineStart.getY());
        }

        double m1 = (lineEnd.getY() - lineStart.getY()) / (lineEnd.getX() - lineStart.getX());
        double x1 = lineStart.getX() - circleCenter.getX();
        double y1 = lineStart.getY() - circleCenter.getY();
        double a = 1 + m1 * m1;
        double b = (2 * m1 * y1) - (2 * m1 * m1 * x1);
        double c = ((Math.pow(m1, 2) * Math.pow(x1, 2)) - (2 * y1 * m1 * x1) + Math.pow(y1, 2) - Math.pow(radius, 2));

        ArrayList<Vector2d> allPoints = new ArrayList<>();

        try {
            double xRoot1 = (-b + Math.sqrt(b * b - 4 * a * c)) / (2 * a);
            double yRoot1 = m1*(xRoot1-x1)+y1;
            //put back the offset
            xRoot1 += circleCenter.getX();
            yRoot1 += circleCenter.getY();
            double minX = Math.min(lineStart.getX(), lineEnd.getX());
            double maxX = Math.max(lineStart.getX(), lineEnd.getX());
            if (xRoot1>minX && xRoot1<maxX) {
                allPoints.add(new Vector2d(xRoot1, yRoot1));
            }
            double xRoot2 = (-b - Math.sqrt(b * b - 4 * a * c)) / (2 * a);
            double yRoot2 = m1*(xRoot2-x1)+y1;
            //put back the offset
            xRoot2 += circleCenter.getX();
            yRoot2 += circleCenter.getY();
            if (xRoot2>minX && xRoot2<maxX) {
                allPoints.add(new Vector2d(xRoot2, yRoot2));
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return allPoints;
    }
}
