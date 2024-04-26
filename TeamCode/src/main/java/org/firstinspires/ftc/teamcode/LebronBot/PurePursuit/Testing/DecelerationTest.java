package org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.Testing;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
    double maxVelocity;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap);
        time=new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        waitForStart();
        time.reset();
        drive.setWeightedDrivePower(new Pose2d(1,0, 0));
        while (opModeIsActive() && !isStopRequested()) {
            if (time.milliseconds()>1000) {
                startStop=time.milliseconds();
                maxVelocity=drive.getVelocity().norm();
                startStopPose=drive.getPoseEstimate();
                drive.setWeightedDrivePower(new Pose2d(0,0, 0));
            }
            if (time.milliseconds()>1000 && drive.getVelocity().norm()<0.1) {
                endStop=time.milliseconds();
                endStopPose=drive.getPoseEstimate();
                break;
            }
            drive.update();
        }
        Log.d("Max Velocity", Double.toString(maxVelocity));
        Log.d("Start Stop Time", Double.toString(startStop));
        Log.d("End Stop Time", Double.toString(endStop));
        Log.d("Predicted Decelration", Double.toString(maxVelocity/(endStop-startStop)));
    }
}
