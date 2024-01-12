package org.firstinspires.ftc.teamcode.jankbot.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;

@TeleOp
public class DriveMotorTest extends OpMode {

    MecanumDrive drive;
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        double a=0, b=0, x=0, y=0;
        if (gamepad1.a) {
            a=0.1;
        }
        if (gamepad1.b) {
            b=0.1;
        }
        if(gamepad1.x) {
            x=0.1;
        }
        if (gamepad1.y) {
            y=0.1;
        }
        drive.setMotorPowers(a,b,x,y);
    }
}
