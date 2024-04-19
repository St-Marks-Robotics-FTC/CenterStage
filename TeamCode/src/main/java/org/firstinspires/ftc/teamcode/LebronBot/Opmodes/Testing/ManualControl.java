package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.Testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;


@Config
@TeleOp
public class ManualControl extends LinearOpMode {

    double v4bPos = 0.5;
    double anglePos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LebronClass robot = new LebronClass(hardwareMap);



        waitForStart();

        robot.intake.tiltDown();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            robot.outtake.setSlidesPower(-gamepad1.left_stick_y);

            if (gamepad1.y && v4bPos < 1) {
                v4bPos += 0.01;
                robot.outtake.setV4Bar(v4bPos);
            } else if (gamepad1.a && v4bPos > 0) {
                v4bPos -= 0.01;
                robot.outtake.setV4Bar(v4bPos);

            }

            if (gamepad1.x && anglePos < 1) {
                anglePos -= 0.01;
                robot.outtake.setV4BarAngle(anglePos);
            } else if (gamepad1.b && anglePos > 0) {
                anglePos += 0.01;
                robot.outtake.setV4BarAngle(anglePos);
            }





            telemetry.update();
        }
    }
}
