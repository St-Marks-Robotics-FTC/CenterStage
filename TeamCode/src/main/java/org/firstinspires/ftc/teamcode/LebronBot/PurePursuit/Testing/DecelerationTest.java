package org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.Testing;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.MecanumDrive;

@Autonomous
public class DecelerationTest extends LinearOpMode {
    MecanumDrive drive;
    ElapsedTime time;
    double startStop;
    Pose2d startStopPose = new Pose2d();
    double endStop;
    Pose2d endStopPose = new Pose2d();
    Vector2d maxVelocity;
    boolean stop = true;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap);
        time=new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        waitForStart();
        time.reset();
        drive.setWeightedDrivePower(new Pose2d(1,0, 0));
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            Log.d("bruh bruh: ", drive.getVelocity().toString());
            if (time.milliseconds()>1500 && stop) {
                startStop=time.milliseconds();
                maxVelocity=drive.getVelocity();
                startStopPose=drive.getPoseEstimate();
                drive.setWeightedDrivePower(new Pose2d(0,0, 0));
                stop = false;
            }
            if (time.milliseconds()>1500 && drive.getVelocity().norm()<0.1) {
                endStop=time.milliseconds();
                endStopPose=drive.getPoseEstimate();
                break;
            }
        }
        Log.d("Max Velocity Norm", Double.toString(maxVelocity.norm()));
        Log.d("Max Velocity", maxVelocity.toString());
        Log.d("Start Stop Time", Double.toString(startStop));
        Log.d("End Stop Time", Double.toString(endStop));
        Log.d("Predicted Decelration", Double.toString(maxVelocity.norm()/(endStop-startStop)*1000.0));
    }
}
