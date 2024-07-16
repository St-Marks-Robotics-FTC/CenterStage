package org.firstinspires.ftc.teamcode.LebronBot.PurePursuit;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Twist2d;

import java.util.ArrayList;
import java.util.List;

public class ErrorCalculator {
    double inPerTick;
    public static double totalDrift;

    public ErrorCalculator() {
        errors[0] =0;
        errors[1] =0;
        totalDrift =0;
    }
    public static double [] errors = new double[]{0.0,0.0};
    Pose2d pose = new Pose2d(0,0,0);

    public void Recalculate(ArrayList<Pose2d> previousPoses, double totalDriftSinceRead) {
        ErrorCalculator.totalDrift = totalDriftSinceRead;
        Log.d("totalDrift: ", Double.toString(Math.toDegrees(ErrorCalculator.totalDrift)));
        //if you just wanted to check difference between estimation and correct data after iterations are complete
        //double latestDW = previousPoses.get(previousPoses.size()-1).get(2);

        //Find the correct heading of each moment
        double numOfIterations = previousPoses.size();

        errors[0]=0;
        errors[1]=0;
        //loop through each estimated pose
        for (int i = 1; i < numOfIterations; i++) {
            double driftRelative_TO_Time = (i / numOfIterations) * totalDriftSinceRead;
            Vector2d rotatedDrift = previousPoses.get(i).vec().rotated(driftRelative_TO_Time);
            Log.d("rotated: ", rotatedDrift.toString());
            errors[0] += rotatedDrift.getX()-previousPoses.get(i).getX();
            errors[1] += rotatedDrift.getY()-previousPoses.get(i).getY();
        }

        previousPoses.clear();
    }

    public double[] FindGlobalError(double[] change) {
        pose = new Pose2d(0, 0, 0);
        double[] localizerValues = new double[2];
        Pose2d twist = new Pose2d(change[0], change[1], change[2]);
        pose = pose.plus(twist);

        localizerValues[0] = pose.getY();
        localizerValues[1] = pose.getX();

        return localizerValues;
    }
}
