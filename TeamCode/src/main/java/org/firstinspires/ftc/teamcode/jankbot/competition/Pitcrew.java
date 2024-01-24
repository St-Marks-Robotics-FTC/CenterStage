package org.firstinspires.ftc.teamcode.jankbot.competition;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.jankbot.Jankbot;

@Disabled
@TeleOp
public class Pitcrew extends OpMode {
    Jankbot robot;
    GamepadEx pad1;

    @Override
    public void init() {
        robot = new Jankbot(hardwareMap);
        pad1 = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        //intake
        if (pad1.isDown(GamepadKeys.Button.A)) {
            robot.intake.setIntake(0.8);
        } else {
            robot.intake.setIntake(0);
        }

        //intake transfer
        if (pad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.intake.tiltUp();
        }
    }
}
