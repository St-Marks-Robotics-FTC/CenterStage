package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;

@Disabled
@Config
@TeleOp
public class LebronFallback extends OpMode {

    LebronClass robot;
    GamepadEx pad1;
    private int slidePos = 0;
    @Override
    public void init() {
        robot = new LebronClass(hardwareMap);
        pad1 = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        if (pad1.isDown(GamepadKeys.Button.DPAD_UP)) {
            robot.closeClaw();
        }
        if (pad1.isDown(GamepadKeys.Button.DPAD_DOWN)) {
            robot.openClaw();
        }
        if (pad1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            slidePos-=10;
        }
        if (pad1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            slidePos+=10;
        }
        if (pad1.wasJustPressed(GamepadKeys.Button.A)) {
            robot.pivot.setPosition(robot.anglePickup);
        }
        if (pad1.wasJustPressed(GamepadKeys.Button.B)) {
            robot.pivot.setPosition(robot.angleStow);
        }
        robot.setSlides(slidePos, robot.getActuatorPosition());

        telemetry.addData("Slide Left: ", robot.slideLeft.getCurrentPosition());
        telemetry.addData("Slide Right: ", robot.slideRight.getCurrentPosition());
        telemetry.addData("Slide Average: ", robot.getSlidePos());
        telemetry.addData("Claw Close: ", robot.clawClose);
        telemetry.addData("Claw Open: ", robot.clawOpen);
        telemetry.addData("Pivot Position: ", robot.getArmPos());
        telemetry.addData("Turret Position: ", robot.turret.getPosition());
    }
}
