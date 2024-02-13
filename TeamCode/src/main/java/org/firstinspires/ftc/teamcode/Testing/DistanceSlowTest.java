package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LM2.LM2class;

@TeleOp
public class DistanceSlowTest extends OpMode {


    DistanceSensor leftDist;
    DistanceSensor rightDist;
    private double maxPow = 1.0;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

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
        leftDist = hardwareMap.get(DistanceSensor.class, "leftDist");
        rightDist = hardwareMap.get(DistanceSensor.class, "rightDist");
    }

    @Override
    public void loop() {
        if (leftDist.getDistance(DistanceUnit.INCH)<6 || rightDist.getDistance(DistanceUnit.INCH)<6) {
            maxPow=0.3;
        } else {
            maxPow=1.0;
        }
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
        backRightMotor.setPower(maxPow*backRightPower);

    }
}
