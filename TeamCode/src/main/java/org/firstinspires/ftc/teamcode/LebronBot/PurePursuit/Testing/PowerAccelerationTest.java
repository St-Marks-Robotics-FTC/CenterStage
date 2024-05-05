package org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.Testing;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.MecanumDrive;

@Autonomous
public class PowerAccelerationTest extends LinearOpMode {
    MecanumDrive drive;
    double power = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        drive=new MecanumDrive(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        timer.reset();
        Vector2d maxAccel = new Vector2d();
        while (!isStopRequested() && opModeIsActive() && timer.milliseconds() < 2000) {
            drive.update();
            drive.setMotorPowers(power, power, power, power);
            Vector2d curAccel = drive.getAccel();
            Log.d("Acceleration: ", curAccel.toString());
            Log.d("Acceleration NORM: ", Double.toString(curAccel.norm()));
            if (curAccel.norm() > maxAccel.norm()) {
                maxAccel = curAccel;
            }

        }
        Log.d("MAX ACCELERATION: ", maxAccel.toString());
        Log.d("MAX ACCELERATION NORM: ", Double.toString(maxAccel.norm()));
    }
}
