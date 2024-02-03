package org.firstinspires.ftc.teamcode.LM2.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LM2.LM2class;
import org.firstinspires.ftc.teamcode.LM2.LM2class;

@TeleOp
public class armbotpitcrew extends OpMode {

    LM2class robot;

    @Override
    public void init() {
        robot = new LM2class(hardwareMap);
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
        telemetry.addData("claw left pos: ", robot.clawLeft.getPosition());
        telemetry.addData("claw right pos: ", robot.clawRight.getPosition());
        telemetry.update();
    }
}
