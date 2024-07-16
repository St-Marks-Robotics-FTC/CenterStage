package org.firstinspires.ftc.teamcode.LebronBot.Roadrunner;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.ErrorCalculator;
import org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.Testing.BackTracking_TUNING;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class Drive extends MecanumDrive {

    Pose2d poseNew;
    double[] lastErrors = new double[3];
    static double [] TotalChange = new double[2];
    double [] prevPose = new double[2];
    double[] newPose= new double []{0.0,0.0};
    static double [] Difference = new double[2];

    //This is the exact percentage of the effectiveness of this Localizer
    double [] percentage_Correction = new double[]{0.0,0.0};
    public IMU imu;
    public static ElapsedTime timer = new ElapsedTime();
    private double heading=0;
    private double lastHeading=0;
    private double headingDelta=0;
    ArrayList<Pose2d> listOfPoses = new ArrayList<>();
    ErrorCalculator errorCalculator;
    Pose2d prevPose2d = new Pose2d();

    public Drive(HardwareMap hardwareMap) {
        super(hardwareMap);
        this.imu = hardwareMap.get(IMU.class, "adafruit_imu");
        this.imu.resetYaw();
        errorCalculator = new ErrorCalculator();
    }

    @Override
    public void update() {
        super.update();
        boolean readImu = timer.milliseconds() >= BackTracking_TUNING.timeBetween_Reads;
//        Pose2d poseVel = getLocalizer().getPoseVelocity();
//        Log.d("poseVel: ", poseVel.toString());
        Pose2d pose = getPoseEstimate();
        Pose2d poseVel = pose.minus(prevPose2d);
        prevPose2d = pose;
        listOfPoses.add(poseVel);
//        Log.d("poseVel: ", poseVel.toString());
        Log.d("pose: ", pose.toString());
        if (readImu && false) {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            heading = angles.getYaw(AngleUnit.RADIANS);
            headingDelta = heading - lastHeading;
            lastHeading=heading;
            double headingDrift = heading - pose.getHeading();
//            Log.d("poseVel: ", poseVel.toString());
            errorCalculator.Recalculate(listOfPoses, headingDrift);
//            Log.d("heading drift: ", Double.toString(Math.toDegrees(headingDrift)));
            timer.reset();
            Pose2d errorCalculator = new Pose2d(ErrorCalculator.errors[0],ErrorCalculator.errors[1],ErrorCalculator.totalDrift);
            Log.d("errorcalculator: ", errorCalculator.toString());
            pose = new Pose2d(pose.getX() + ErrorCalculator.errors[0], pose.getY() + ErrorCalculator.errors[1], pose.getHeading() + ErrorCalculator.totalDrift);
//            lastErrors = new double[]{ErrorCalculator.errors[0], ErrorCalculator.errors[1], ErrorCalculator.totalDrift};
            listOfPoses.clear();
        }
        setPoseEstimate(pose);
        if (readImu) {
            Log.d("final pose: ", pose.toString());
        }
    }

    public double [] TotalMovement(){
        double [] positions = new double[]{getPoseEstimate().getY(),getPoseEstimate().getX()};
        for (int i =0; i<2; i++){
            prevPose[i] = newPose[i];
            newPose[i] = positions[i];
            Difference[i] = Math.abs(newPose[i] - prevPose[i]);
            TotalChange[i] += Difference[i];
        }

        if(TotalChange[0] != 0 && TotalChange[1]!= 0){
            for (int i =0; i<2; i++){
                percentage_Correction[i] = 100*Math.abs(ErrorCalculator.errors[i])/TotalChange[i];
            }
        }

        return percentage_Correction;
    }
}
