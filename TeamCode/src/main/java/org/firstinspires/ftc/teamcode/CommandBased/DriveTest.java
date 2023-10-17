package org.firstinspires.ftc.teamcode.CommandBased;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.DoubleSupplier;

@TeleOp
public class DriveTest extends OpMode {

    MecanumCommandBase mec;
    MecanumSubsystem chassis;

    String FL = "frontLeft";
    String BL = "backLeft";
    String FR = "frontRight";
    String BR = "backRight";

    private static double driveMult = 1.0;

    @Override
    public void init() {
        chassis = new MecanumSubsystem(hardwareMap, FL, BL, FR, BR);
        chassis.brake();
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            driveMult = 0.3;
        } else {
            driveMult = 1.0;
        }
        final DoubleSupplier y = () -> gamepad1.left_stick_y*driveMult;
        final DoubleSupplier x = () -> -gamepad1.left_stick_x*driveMult;
        final DoubleSupplier rx = () -> -gamepad1.right_stick_x*driveMult;
        telemetry.addData("gamepad1.leftstick", gamepad1.left_stick_y);
        mec = new MecanumCommandBase(chassis,y, x, rx);
        mec.execute();
    }
}
