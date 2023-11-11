package org.firstinspires.ftc.teamcode.armbot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class motorTest extends OpMode {

    DcMotor motor;
    public static String motorName = "arm";

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get(motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            motor.setPower(0.3);
        } else if (gamepad1.dpad_down) {
            motor.setPower(-0.3);
        } else {
            motor.setPower(0);
        }
        telemetry.addData("Motor pos: ", motor.getCurrentPosition());
        telemetry.update();
    }
}
