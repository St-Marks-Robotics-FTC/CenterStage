package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.Testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;


@Config
@TeleOp
public class OuttakePos extends LinearOpMode {

    double v4bPos = 0.5;
    double anglePos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LebronClass robot = new LebronClass(hardwareMap);



        waitForStart();

        robot.intake.tiltDown();
        robot.outtake.turretTransfer();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            robot.outtake.setSlidesPower(-gamepad1.left_stick_y);


            if (gamepad1.x) { // regular stow pos
                robot.outtake.setV4Bar(0.19);
//                robot.outtake.setV4BarAngle(0.7);
            } else if (gamepad1.y) { // vertical
                robot.outtake.setV4Bar(0.45);
//                robot.outtake.setV4BarAngle(0.35);
            } else if (gamepad1.b) { // right (score)
                robot.outtake.setV4Bar(0.72);
//                robot.outtake.setV4BarAngle(0.03);
            } else if (gamepad1.a) { // fully done (dead)
                robot.outtake.setV4Bar(0.9);
//                robot.outtake.setV4BarAngle(0);
            }


            telemetry.update();
        }
    }
}
