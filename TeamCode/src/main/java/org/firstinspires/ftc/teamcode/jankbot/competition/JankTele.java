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
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;
            if (gamepad1.left_bumper) {
                robot.drive.setWeightedDrivePower(new Pose2d(x, y, rx));
            } else {
                robot.drive.setWeightedDrivePower(new Pose2d(0.4 * x, 0.4 * y, 0.3 * rx));
            }




            loopTime = time.milliseconds();
            telemetry.addData("Hz", 1000 / (loopTime - prevTime) );
            prevTime = loopTime;

            telemetry.update();
            pad1.readButtons();

        }
    }
}