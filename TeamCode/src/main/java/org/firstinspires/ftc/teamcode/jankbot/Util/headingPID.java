package org.firstinspires.ftc.teamcode.jankbot.Util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;

@TeleOp
public class headingPID extends OpMode {

    MecanumDrive drive;
    private double turn = 0.0;
    private double x = 0;
    private double y = 0;
    private double h = 0;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0,0));
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            turn=drive.getPoseEstimate().getHeading();
        }
        if (gamepad1.b) {
            drive.turnAsync(turn);
        }
        x=gamepad1.left_stick_x;
        y= gamepad1.right_stick_y;
        drive.setWeightedDrivePower(new Pose2d(x*10, y*10, h));
        drive.update();
    }
}
