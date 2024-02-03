package org.firstinspires.ftc.teamcode.Fallback.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.jankbot.Jankbot;

@Disabled
@Config
@TeleOp (group = "test")
public class intakeTest extends OpMode {

    Jankbot robot;
    GamepadEx pad1;
    double power=0.0;
    @Override
    public void init() {
        robot = new Jankbot(hardwareMap);
        pad1 = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        if (pad1.isDown(GamepadKeys.Button.A)) {
            power+=0.01;
        } else if (pad1.isDown(GamepadKeys.Button.B)) {
            power-=0.01;
        }
        robot.intake.setIntake(power);
        telemetry.addData("intake power: ", power);
        telemetry.update();
    }
}
