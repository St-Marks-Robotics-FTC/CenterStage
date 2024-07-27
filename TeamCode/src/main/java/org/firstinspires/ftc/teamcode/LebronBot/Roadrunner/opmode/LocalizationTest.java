package org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.MecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */

//@Disabled

@Config
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    double loopTime = 0;
    Pose2d prev = new Pose2d();
    Pose2d total = new Pose2d();
    @Override
    public void runOpMode() throws InterruptedException {
//        Drive2 drive = new Drive2(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        prev = drive.getPoseEstimate();

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x*0.6
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            total=new Pose2d(total.getX()+poseEstimate.getX()-prev.getX(),total.getY()+poseEstimate.getY()-prev.getY(),total.getHeading()+poseEstimate.getHeading()-prev.getHeading());
            prev=new Pose2d(poseEstimate.vec(), poseEstimate.getHeading());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            double loop = System.nanoTime();
            double hz = 1000000000 / (loop - loopTime);
            Log.d("hz ", Double.toString(1000000000 / (loop - loopTime)));
            loopTime = loop;

            telemetry.update();
        }
        Log.d("total: ", total.toString());
    }
}
