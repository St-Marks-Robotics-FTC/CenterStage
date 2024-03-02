package org.firstinspires.ftc.teamcode.Testing.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Testing")
public class ChassisTest extends OpMode {


    DistanceSensor leftDist;
    DistanceSensor rightDist;
    private double maxPow = 1.0;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    DcMotor intake;
    DcMotor slide;

    Servo intakeAngle;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        slide = hardwareMap.dcMotor.get("slide");

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeAngle = hardwareMap.servo.get("pivot");

    }

    @Override
    public void loop() {
        double tranScaleFactor = gamepad1.left_bumper ? 0.4 : 1.0;
        double rotScaleFactor = gamepad1.left_bumper ? 0.3 : 1.0;

        double y = -gamepad1.left_stick_y * tranScaleFactor;
        double x = gamepad1.left_stick_x * tranScaleFactor;
        double rx = gamepad1.right_stick_x * rotScaleFactor;



        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator * tranScaleFactor;
        double backLeftPower = (y - x + rx) / denominator * tranScaleFactor;
        double frontRightPower = (y - x - rx) / denominator * tranScaleFactor;
        double backRightPower = (y + x - rx) / denominator * tranScaleFactor;

        frontLeftMotor.setPower(maxPow*frontLeftPower);
        backLeftMotor.setPower(maxPow*backLeftPower);
        frontRightMotor.setPower(maxPow*frontRightPower);
        backRightMotor.setPower(maxPow* backRightPower);

        intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);



        if (gamepad1.a) {
            intakeAngle.setPosition(0.51);
        } else if (gamepad1.y) {
            intakeAngle.setPosition(0.2);
        } else if (gamepad1.b) {
            intakeAngle.setPosition(0.445);
        }

        if (gamepad1.dpad_up) {
            slide.setTargetPosition(485);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
        } else if (gamepad1.dpad_down) {
            slide.setTargetPosition(5);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
        }




        telemetry.addData("slide pos", slide.getCurrentPosition());
        telemetry.update();
    }
}
