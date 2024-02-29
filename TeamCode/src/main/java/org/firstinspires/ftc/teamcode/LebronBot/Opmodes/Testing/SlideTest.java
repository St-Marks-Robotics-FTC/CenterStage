package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled

@Config
@TeleOp
public class SlideTest extends OpMode {

    DcMotorEx slideLeft;
    DcMotorEx slideRight;

    private double power = 1.0;
    private int position = 700;

    @Override
    public void init() {
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        slideLeft.setDirection(DcMotorEx.Direction.REVERSE);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            slideLeft.setPower(power);
            slideRight.setPower(power);
        }
        if (gamepad1.b) {
            slideLeft.setPower(-0.5);
            slideRight.setPower(-0.5);
        }
        slideLeft.setPower(0);
        slideRight.setPower(0);
        if (gamepad1.x) {
            slideLeft.setTargetPosition(position);
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideLeft.setPower(power);
            slideRight.setTargetPosition(position);
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setPower(power);
        } else {
            slideLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            slideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (gamepad1.dpad_down) {
            slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        telemetry.addData("Slide Left: ", slideLeft.getCurrentPosition());
        telemetry.addData("Slide Right: ", slideRight.getCurrentPosition());
        telemetry.update();
    }
}
