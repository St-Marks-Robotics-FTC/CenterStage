package org.firstinspires.ftc.teamcode.Testing.CommandBased;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DriveTest extends OpMode {

    MecanumSubsystem chassis;


    @Override
    public void init() {
        chassis = new MecanumSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        chassis.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_bumper);
    }
}
