package org.firstinspires.ftc.teamcode.CommandBased;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.DoubleSupplier;

@TeleOp
public class DriveTest extends OpMode {

    MecanumCommandBase mec;
    MecanumSubsystem chassis;

    String FL = "leftFront";
    String BL = "leftBack";
    String FR = "rightFront";
    String BR = "rightBack";


    @Override
    public void init() {
        chassis = new MecanumSubsystem(hardwareMap, FL, BL, FR, BR);
        //chassis.brake();
    }

    @Override
    public void loop() {
        final DoubleSupplier y = () -> gamepad1.left_stick_y;
        final DoubleSupplier x = () -> -gamepad1.left_stick_x;
        final DoubleSupplier rx = () -> -gamepad1.right_stick_x;
        telemetry.addData("gamepad1.leftstick", gamepad1.left_stick_y);
        mec = new MecanumCommandBase(chassis,y, x, rx);
        mec.execute();
    }
}
