package org.firstinspires.ftc.teamcode.armbot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class servoTest extends OpMode {

    Servo servo;
    public static String servoName = "claw";
    public static double position = 0.5;

    @Override
    public void init() {
        servo = hardwareMap.servo.get(servoName);
    }

    @Override
    public void loop() {
        servo.setPosition(position);
        telemetry.addData("Servo pos: ", position);
        telemetry.update();
    }
}
