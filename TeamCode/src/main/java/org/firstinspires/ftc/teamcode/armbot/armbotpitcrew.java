package org.firstinspires.ftc.teamcode.armbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class armbotpitcrew extends OpMode {

    Arm robot;

    @Override
    public void init() {
        robot = new Arm(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            robot.arm.setPower(0.3);
        }
        if (gamepad1.dpad_down) {
            robot.arm.setPower(-0.3);
        }
        if (gamepad1.a) {
            robot.closeClaw();
        }
        if (gamepad1.b) {
            robot.openClaw();
        }
        telemetry.addData("arm pos: ", robot.arm.getCurrentPosition());
        telemetry.addData("claw closed: ", robot.claw.getPosition());
        telemetry.update();
    }
}
