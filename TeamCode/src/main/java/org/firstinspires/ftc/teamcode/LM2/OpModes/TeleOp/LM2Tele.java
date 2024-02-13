package org.firstinspires.ftc.teamcode.LM2.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LM2.LM2class;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.MecanumDrive;

@Config
@TeleOp
public class LM2Tele extends LinearOpMode {

    // Arm
    public static int armDown = 0;
    public static int armUp = 350;
    public static int armUp2 = 405; //CHANGE
    public static int armUp3 = 480; //CHANGE
    public static int[] armPos = {armDown,armUp2,armUp3};
    public static int level = 0;

    public static boolean dpadupPressed = false;
    public static int[] hangPos = {970,1850}; //CHANGE

    // Drone
    public static boolean droneLaunch = false;

    // Claw
    public static boolean closed = false;
    public static boolean leftClosed = false;
    public static boolean rightClosed = false;
    public static boolean prevLeftClosed = false;
    public static boolean prevRightClosed = false;

    public static LM2class robot;
    MecanumDrive drive;
    public static GamepadEx pad1;


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

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);


        robot = new LM2class(hardwareMap);
        drive = new MecanumDrive(hardwareMap);
        pad1 = new GamepadEx(gamepad1);
        TriggerReader droneTrigger = new TriggerReader(
                pad1, GamepadKeys.Trigger.LEFT_TRIGGER
        );

        ElapsedTime time = new ElapsedTime();

        dpadupPressed = false; // So arm doesnt flip all the way w
        droneLaunch = false;
        robot.openClaw();
        robot.closeDrone();
        waitForStart();
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
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

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);




            if (pad1.wasJustPressed(GamepadKeys.Button.Y)) {
                level = Math.min(2, level+1);
                robot.setArm(armPos[level]);
            } else if (pad1.wasJustPressed(GamepadKeys.Button.A)/* && !closed*/) {
                if (!closed) {
                    level = 0; // goes down bypassing level 1 when already scored
                } else {
                    level = Math.max(0, level-1);
                }
                robot.setArm(armPos[level]);

                if (!closed && level == 0) {
                    robot.openClaw();
                } // makes score claw to open claw
            } else if (pad1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.setArm(robot.arm.getCurrentPosition()+15);
            } else if(gamepad1.right_trigger>=0.3){
                robot.setArm(robot.arm.getCurrentPosition()-20);
            }

            // Drone
            if (droneTrigger.wasJustPressed()) {
                droneLaunch = !droneLaunch;
            }
            if (droneLaunch) {
                robot.openDrone();
            } else {
                robot.closeDrone();
            }

            // Claw
            if (pad1.wasJustPressed(GamepadKeys.Button.X)) {
                robot.closeClaw();
                leftClosed = true;
                rightClosed = true;
                prevLeftClosed = true;
                prevRightClosed = true;

                closed = true;
            } else if (pad1.wasJustPressed(GamepadKeys.Button.B)) {
                if (level >= 1) {
                    robot.scoreClaw(); // slightly open
                } else {
                    robot.openClaw(); // fully open
                }
                leftClosed = false;
                rightClosed = false;
                closed = false;
            }
            if (pad1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { // left claw toggle
                if (!leftClosed){
                    robot.closeLeft();
                    leftClosed = true;
                } else {
                    robot.openLeft();
                    rightClosed = false;
                }
            } else if (pad1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { // right claw toggle
                if (!rightClosed){
                    robot.closeRight();
                    rightClosed = true;
                } else {
                    robot.openRight();
                    rightClosed = false;
                }
            }
            if (prevLeftClosed) {
                gamepad1.rumble(1.0, 0.0, 1000);
                prevLeftClosed=false;
            }
            if (prevRightClosed) {
                gamepad1.rumble(0.0, 1.0, 1000);
                prevRightClosed=false;
            }
            if (!rightClosed && !leftClosed) closed = false; // if both are open, claw is open
            else if (rightClosed && leftClosed) closed = true; // if both are closed, claw is closed




            // Hang
            if(pad1.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                if(!dpadupPressed){
                    robot.setArm(hangPos[0]);
                    dpadupPressed = true;
                } else {
                    robot.hang(hangPos[1]);
                    dpadupPressed=false;
                }
            }





            pad1.readButtons();
            droneTrigger.readValue();


            telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
            telemetry.addData("Arm Target", robot.arm.getTargetPosition());
            telemetry.addData("Arm Power", robot.arm.getPower());

            telemetry.addData("Claw Closed", closed);
            telemetry.addData("Left Claw closed", leftClosed);
            telemetry.addData("Right Claw closed", rightClosed);
            telemetry.update();



        }
    }
}