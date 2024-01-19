package org.firstinspires.ftc.teamcode.mentorbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MecanumTeleop extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    DcMotor arm;
    Servo wrist;
    Servo leftClaw;
    Servo rightClaw;

    Servo drone;

    double leftOpen = 0.2;
    double rightOpen = 0.2;
    double leftClose = 0.6;
    double rightClose = 0.6;

    int armLevel = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();



        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");

        arm = hardwareMap.dcMotor.get("arm");
        wrist = hardwareMap.servo.get("wrist");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");

        drone = hardwareMap.servo.get("drone");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        drone.setPosition(0.5); // hold
        waitForStart();

        if (isStopRequested()) return;
        boolean up = false;
        while (opModeIsActive()) {
            // Store the gamepad values from the previous loop iteration in
            // previousGamepad1/2 to be used in this loop iteration.
            // This is equivalent to doing this at the end of the previous
            // loop iteration, as it will run in the same order except for
            // the first/last iteration of the loop.
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Store the gamepad values from this loop iteration in
            // currentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents the gamepad values from changing between being
            // used and stored in previousGamepad1/2.
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);





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

            if (gamepad1.left_bumper) {
                frontLeftMotor.setPower(0.35 * frontLeftPower);
                backLeftMotor.setPower(0.35 * backLeftPower);
                frontRightMotor.setPower(0.35 * frontRightPower);
                backRightMotor.setPower(0.35 * backRightPower);
            } else {
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }


            if (gamepad1.y) {
                up = true;
            } else if (gamepad1.a) {
                up = false;
            }

            if (up) {
                if (armLevel == 1) {
                    wrist.setPosition(0.4);
                    moveArm(300, 0.4);
                } else if (armLevel == 2) {
                    wrist.setPosition(0.6);
                    moveArm(500, 0.4);
                } else if (armLevel == 3) {
                    wrist.setPosition(0.7);
                    moveArm(700, 0.4);
                }
            } else {
                wrist.setPosition(0.1);
                moveArm(0, 0.4);
            }

            if (!previousGamepad2.dpad_up && currentGamepad2.dpad_up) {
                armLevel = Math.min(3, armLevel + 1);
            } else if (!previousGamepad2.dpad_down && currentGamepad2.dpad_down) {
                armLevel = Math.max(0, armLevel - 1);
            }


            if (gamepad1.x) { // close claw
                closeClaw();
            } else if (gamepad1.b) { // open claw
                openClaw();
            } else if (gamepad2.x) { // open left
                openLeft();
            } else if (gamepad2.b) { // open right
                openRight();
            }




            if (gamepad2.y) {
                drone.setPosition(0.1); // launch
            }


            telemetry.addData("Arm Level", armLevel);
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Wrist Position", wrist.getPosition());

            telemetry.update();

        }
    }

    public void moveArm(int position, double speed) {
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(speed);
    }

    public void openClaw() {
        leftClaw.setPosition(leftOpen);
        rightClaw.setPosition(rightOpen);
    }
    public void openLeft() {
        leftClaw.setPosition(leftOpen);
    }
    public void openRight() {
        rightClaw.setPosition(rightOpen);
    }
    public void closeClaw() {
        leftClaw.setPosition(leftClose);
        rightClaw.setPosition(rightClose);
    }
}
