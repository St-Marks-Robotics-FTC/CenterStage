package org.firstinspires.ftc.teamcode.jankbot.competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.armbot.BozoClass;
import org.firstinspires.ftc.teamcode.jankbot.Jankbot;

@Config
@TeleOp
public class JankTele extends LinearOpMode {


    double loopTime = 0;
    double prevTime = 0;

    Jankbot robot;
    GamepadEx pad1;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Jankbot(hardwareMap);
        pad1 = new GamepadEx(gamepad1);
        ElapsedTime time = new ElapsedTime();




        waitForStart();
        time.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double tranScaleFactor = gamepad1.left_bumper ? 0.4 : 1.0;
            double rotScaleFactor = gamepad1.left_bumper ? 0.3 : 1.0;
            robot.drive.setWeightedDrivePower(
                    new Pose2d(tranScaleFactor * y, tranScaleFactor * x, rotScaleFactor * 0.3 * rx)
            );


















            robot.drive.update();

            Pose2d poseEstimate = robot.drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());


            loopTime = time.milliseconds();
            telemetry.addData("Hz", 1000 / (loopTime - prevTime) );
            prevTime = loopTime;

            telemetry.update();
            pad1.readButtons();

        }
    }
}