package org.firstinspires.ftc.teamcode.Testing.Misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.DriveConstants;

//@Disabled
@Config
@TeleOp
public class HeadingLock extends LinearOpMode {
    private PIDController controller;

    // The IMU sensor object
    private IMU imu;


    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static double targetAngle;
    double currAngle;

    boolean lock = false;


    ElapsedTime lockTimer = new ElapsedTime();

    public static double lockTime = 0.5;


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
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);



        waitForStart();
        targetAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        lockTimer.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);

            currentGamepad1.copy(gamepad1);




            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;




            // heading lock pid






            if (currentGamepad1.right_stick_x != 0) {
                lock = false;
            } else if (currentGamepad1.right_stick_x == 0 && previousGamepad1.right_stick_x != 0) { // falling edge
                lockTimer.reset();
            } else if (lockTimer.seconds() > lockTime && !lock) {
                targetAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                lock = true;
            }

            controller.setPID(p, i, d);
            currAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double pid = controller.calculate(currAngle, targetAngle);

            double power = pid + f;


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]



            // Using the toggle variable to control the robot.
            if (lock) { // PID

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx) + Math.abs(power), 1);
                double frontLeftPower = (y + x + rx - power) / denominator;
                double backLeftPower = (y - x + rx - power) / denominator;
                double frontRightPower = (y - x - rx + power) / denominator;
                double backRightPower = (y + x - rx + power) / denominator;

                frontLeftMotor.setPower(frontLeftPower); // positive power = clockwise
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }
            else { // normal
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower); // positive power = clockwise
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }




            //motor.setPower(power);

            telemetry.addData("Heading Lock: ", lock);
            telemetry.addData("Lock Timer: ", lockTimer.seconds());
            telemetry.addData("Lock Time: ", lockTime);

            telemetry.addLine("////////////////////////////");

            telemetry.addData("Current Angle: ", currAngle);
            telemetry.addData("target angle: ", targetAngle);

            telemetry.addLine("////////////////////////////");

            telemetry.addData("p: ", p);
            telemetry.addData("i: ", i);
            telemetry.addData("d: ", d);
            telemetry.addData("power: ", power);
            telemetry.update();


        }
    }
}