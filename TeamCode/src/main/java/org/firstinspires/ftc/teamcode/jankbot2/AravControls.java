package org.firstinspires.ftc.teamcode.jankbot2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.jankbot2.jankbot2Class;

@Config
@Disabled
@TeleOp
public class NewControls extends LinearOpMode {

    public static int armDown = 0;
    public static int armUp = 350;
    public static int armUp2 = 425; //CHANGE
    public static int armUp3 = 505; //CHANGE
    public static int[] armPos = {armDown,armUp2,armUp3};
    public static int level = 0;

    public static boolean dpadupPressed = false;
    public static int[] hangPos = {970,1850}; //CHANGE

    public static boolean closed = false;
    public static boolean leftClosed = false;
    public static boolean rightClosed = false;

    double loopTime = 0;
    double prevTime = 0;

    public static jankbot2Class robot;
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


        robot = new jankbot2Class(hardwareMap);
        pad1 = new GamepadEx(gamepad1);
        TriggerReader leftTrigger = new TriggerReader(
                pad1, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        TriggerReader rightTrigger = new TriggerReader(
                pad1, GamepadKeys.Trigger.RIGHT_TRIGGER
        );

        ElapsedTime time = new ElapsedTime();

        robot.openClaw();
        waitForStart();
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

            if (gamepad1.left_bumper) {
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




            if (level == 0) { // arm currently down
                if (leftTrigger.wasJustPressed()) { // CLAW OPEN
                    robot.openClaw();
                    closed = false;
                } else if (rightTrigger.wasJustPressed()) {  // CLAW CLOSE
                    robot.closeClaw();
                    closed = true;
                    leftClosed = true;
                    rightClosed = true;
                }

                if (pad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && closed) { // ARM UP
                    level = 1;
                    robot.setArm(armPos[level]);
                }

            } else {

                if (leftTrigger.wasJustPressed()) { // LEFT OPEN
                    robot.openLeft();
                    leftClosed = false;
                } else if (rightTrigger.wasJustPressed()) {  // RIGHT CLOSE
                    robot.openRight();
                    rightClosed = false;
                }

                if (pad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) { // ARM Toggle
                    if (!leftClosed && !rightClosed) {
                        closed = false;
                        level = 0;
                    } else if (level == 1) {
                        level = 2;
                    } else if (level == 2){
                        level = 1;
                    }
                    robot.setArm(armPos[level]);
                }

            }


//            if(pad1.wasJustPressed(GamepadKeys.Button.Y)){ // HANG
//                if(!dpadupPressed){
//                    robot.setArm(hangPos[0]);
//                    dpadupPressed = true;
//                } else {
//                    robot.hang(hangPos[1]);
//                    dpadupPressed=false;
//                }
//            }



            pad1.readButtons();
            leftTrigger.readValue();
            rightTrigger.readValue();

            loopTime = time.milliseconds();
            telemetry.addData("Hz", 1000 / (loopTime - prevTime) );
            prevTime = loopTime;

            telemetry.addData("Arm Level", level);
            telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
            telemetry.addData("Arm Power", robot.arm.getPower());

//            telemetry.addData("Left Trigger", leftTrigger.wasJustPressed());
//            telemetry.addData("Left Trigger value", pad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

            telemetry.addData("Left Claw closed", leftClosed);
            telemetry.addData("Right Claw closed", rightClosed);
            //telemetry.addData("Current", robot.arm.getCurrent());
            telemetry.update();

        }
    }
}