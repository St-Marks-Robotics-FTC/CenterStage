package org.firstinspires.ftc.teamcode.Testing.Misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.DriveConstants;
import org.opencv.core.Mat;

//@Disabled
@Config
@TeleOp(group = "Testing")
public class AutoAlign extends LinearOpMode {
    private PIDController controller;

    // The IMU sensor object
    private IMU imu;


    public static double p = 0.7, i = 0, d = 0.05;
    public static double f = 0;

    public static double targetAngle = Math.toRadians(0);
    double currAngle;

    boolean PID = false;

    double pidPower = 0;
    double correctionPower = 0;

    boolean stickZero = false;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // By setting these values to new Gamepad(), they will default to all
        // boolean values as false and all float values as 0
        Gamepad currentGamepad1 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();




        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);




        // PID
        controller = new PIDController(p, i , d);

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(IMU.class, "adafruit_imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);

            currentGamepad1.copy(gamepad1);


            currAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//            currAngle = AngleUnit.normalizeRadians(currAngle);

            if (gamepad1.dpad_down) {
                targetAngle = currAngle;
            }




            double tranScaleFactor = gamepad1.left_bumper ? 0.4 : 1.0;
            double rotScaleFactor = gamepad1.left_bumper ? 0.4 : 0.9;

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            // Apply scaling factors
            y *= tranScaleFactor;
            x *= tranScaleFactor;

            if (gamepad1.right_stick_button || gamepad1.y) {
                PID = true;
                stickZero = false;
            } else if (gamepad1.a) {
                PID = false;
            }

            double rx;

            if (PID) {
                if (Math.abs(gamepad1.right_stick_x) < 0.01) {
                    stickZero = true;
                }
                if (Math.abs(gamepad1.right_stick_x) > 0.1 && stickZero) {
                    PID = false;
                }


                controller.setPID(p, i, d);
                pidPower = controller.calculate(AngleUnit.normalizeRadians(currAngle - targetAngle), 0);

                correctionPower = pidPower + f;
//                if (controller.getPositionError() < Math.toRadians(5)) { // 5 degree tolerace
//                    correctionPower = 0;
//                }
                rx = -correctionPower;
            } else {
                rx = gamepad1.right_stick_x;
                rx *= rotScaleFactor;
            }




            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower); // positive power = clockwise
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);




            //motor.setPower(power);

            telemetry.addData("Align on: ", PID);

            telemetry.addLine("////////////////////////////");

            telemetry.addData("Current Angle: ", Math.toDegrees(currAngle));
            telemetry.addData("target angle: ", Math.toDegrees(targetAngle));

            telemetry.addLine("////////////////////////////");

            telemetry.addData("p: ", p);
            telemetry.addData("i: ", i);
            telemetry.addData("d: ", d);
            telemetry.addData("f: ", f);
            telemetry.addData("power: ", pidPower);

            telemetry.addLine("////////////////////////////");
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Right Stick X", gamepad1.right_stick_x);

            telemetry.update();


        }
    }
}