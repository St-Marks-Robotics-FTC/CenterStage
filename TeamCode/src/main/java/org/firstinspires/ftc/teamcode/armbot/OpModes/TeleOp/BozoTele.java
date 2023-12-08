package org.firstinspires.ftc.teamcode.armbot.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.armbot.BozoClass;

@Config
@TeleOp
public class BozoTele extends LinearOpMode {

    public static double clawOpen = 0.35; // 0.3
    public static double clawClosed = 0.41;

    public static int armUp = 453;
    public static int armUp2 = 553; //CHANGE
    public static int armUp3 = 663; //CHANGE
    public static int armDown = 0;
    public static int[] armPos = {armDown,armUp,armUp2,armUp3};
    public static int level = 0;
    public static boolean closed = false;
    public static boolean leftClosed = false;
    public static boolean rightClosed = false;
    public static int[] hangPos = {750,800}; //CHANGE
    public static boolean dpadupPressed = false;
    public static BozoClass robot;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


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

        //DcMotor armMotor = hardwareMap.dcMotor.get("arm");
        ElapsedTime time = new ElapsedTime();
        // claw servo
        //Servo clawServo = hardwareMap.servo.get("claw");
        robot = new BozoClass(hardwareMap);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // reset arm
        //armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        armMotor.setPower(-0.05);
        waitForStart();
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.left_trigger>=0.3) {
                frontLeftMotor.setPower(0.3 * frontLeftPower);
                backLeftMotor.setPower(0.3 * backLeftPower);
                frontRightMotor.setPower(0.3 * frontRightPower);
                backRightMotor.setPower(0.3 * backRightPower);
            } else {
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }


            if(gamepad1.dpad_up){
                if(!dpadupPressed){
                    robot.setArm(hangPos[0]);
                    dpadupPressed = true;
                }
                else{
                    robot.setArm(hangPos[1]);
                }
            }
            if (gamepad1.y) {
                level = Math.min(3, level+1);
                robot.setArm(armPos[level]);
            } else if (gamepad1.a && !closed) {
                level = Math.max(0, level-1);
                robot.setArm(armPos[level]);
            } else if (gamepad1.right_bumper) {
                robot.setArm(robot.arm.getCurrentPosition()+30);
            } else if(gamepad1.left_bumper){
                robot.setArm(Math.max(robot.arm.getCurrentPosition()-30,0));
            }

            if (gamepad1.x) {
                robot.closeClaw();
                closed = true;
            } else if (gamepad1.b) {
                robot.openClaw();
                closed = false;
            }
            if (gamepad1.dpad_left) {
                if (!leftClosed){
                    robot.closeLeft();
                    leftClosed = true;
                }
                else{
                    robot.openLeft();
                    rightClosed = false;
                }
            }
            if (gamepad1.dpad_right) {
                if (!rightClosed){
                    robot.closeRight();
                    rightClosed = true;
                }
                else{
                    robot.openRight();
                    rightClosed = false;
                }
            }

            telemetry.addData("arm position", robot.arm.getCurrentPosition());
            telemetry.addData("left claw closed or not", leftClosed);
            telemetry.addData("right claw closed or not", rightClosed);
            telemetry.update();

        }
    }
}