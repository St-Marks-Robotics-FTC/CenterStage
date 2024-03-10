package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(group = "Testing")
public class ChassisV2 extends LinearOpMode {

    public enum LinearStates {
        IDLE,
        DOWN,
        UP,
        TRANSFER,
        GRAB,
        DROP,
        FLIP,
        IDLE2,
        SCORE,
        RESET,


        OUTTAKE
    }

    boolean pid = true;

    int level = 1;
    boolean up = false;
    String pivot = "ground";

    GamepadEx pad1;
    TriggerReader leftTrigger;
    TriggerReader rightTrigger;

    DistanceSensor leftDist;
    DistanceSensor rightDist;
    private double maxPow = 1.0;
    DcMotorEx frontLeftMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backRightMotor;

    DcMotorEx leftActuator;
    DcMotorEx rightActuator;

    DcMotorEx intake;
    DcMotorEx slide;

    Servo intakeAngle;

    Servo arm;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);


        leftActuator = hardwareMap.get(DcMotorEx.class, "leftActuator");
        rightActuator = hardwareMap.get(DcMotorEx.class, "rightActuator");

//        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        intake.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        claw = hardwareMap.servo.get("claw");
//        arm = hardwareMap.servo.get("arm");
//
//        slide = hardwareMap.dcMotor.get("slide");
//
//        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slide.setDirection(DcMotorSimple.Direction.REVERSE);
//        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        intakeAngle = hardwareMap.servo.get("pivot");

        pad1 = new GamepadEx(gamepad1);
        leftTrigger = new TriggerReader(
                pad1, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        rightTrigger = new TriggerReader(
                pad1, GamepadKeys.Trigger.RIGHT_TRIGGER
        );



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


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
            double frontRightPower = (y - x - rx) / denominator * tranScaleFactor;
            double backLeftPower = (y - x + rx) / denominator * tranScaleFactor;
            double backRightPower = (y + x - rx) / denominator * tranScaleFactor;

            frontLeftMotor.setPower(maxPow * frontLeftPower);
            frontRightMotor.setPower(maxPow * frontRightPower);
            backLeftMotor.setPower(maxPow * backLeftPower);
            backRightMotor.setPower(maxPow * backRightPower);

//            // Intake height mode
//            if (pad1.wasJustPressed(GamepadKeys.Button.A)) {
//                pivot = "ground";
//            } else if (pad1.wasJustPressed(GamepadKeys.Button.B)) {
//                pivot = "stack";
//            }
//
//            // Slide level adjust
//            if (pad1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//                level = Math.min(level + 1, 4);
//            } else if (pad1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//                level = Math.max(level - 1, 1);
//            }


//            if (pad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
//                up = !up;
//            }
//            if (up) {
//                slide.setTargetPosition(485);
//                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slide.setPower(1);
//            } else {
//                slide.setTargetPosition(5);
//                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slide.setPower(1);
//            }



            if (gamepad1.a) {
                pid = true;
                leftActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                rightActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

                leftActuator.setTargetPosition(0);
                leftActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftActuator.setPower(1);
                rightActuator.setTargetPosition(0);
                rightActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightActuator.setPower(1);
            } else if (gamepad1.y) {
                pid = true;
                leftActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                rightActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

                leftActuator.setTargetPosition(8000);
                leftActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftActuator.setPower(1);
                rightActuator.setTargetPosition(8000);
                rightActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightActuator.setPower(1);
            } else if (gamepad1.left_trigger > 0.1) {
                pid = false;
                leftActuator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                rightActuator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

                leftActuator.setPower(-gamepad1.left_trigger);
                rightActuator.setPower(-gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.1) {
                pid = false;
                leftActuator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                rightActuator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

                leftActuator.setPower(gamepad1.right_trigger);
                rightActuator.setPower(gamepad1.right_trigger);
            } else if (!pid){
                leftActuator.setPower(0);
                rightActuator.setPower(0);
            }

            if (gamepad1.dpad_down) { // reset encoders
                leftActuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                rightActuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                leftActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                rightActuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }



            pad1.readButtons();
            leftTrigger.readValue();
            rightTrigger.readValue();

            telemetry.addData("Forward power", gamepad1.left_stick_y);

            telemetry.addData("FL Current", frontLeftMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("FR Current", frontRightMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("BL Current", backLeftMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("BR Current", backRightMotor.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("FL Speed", frontLeftMotor.getVelocity());
            telemetry.addData("FR Speed", frontRightMotor.getVelocity());
            telemetry.addData("BL Speed", backLeftMotor.getVelocity());
            telemetry.addData("BR Speed", backRightMotor.getVelocity());


            telemetry.addData("Left Actuator Pos: ", leftActuator.getCurrentPosition());
            telemetry.addData("Right Actuator Pos: ", rightActuator.getCurrentPosition());
            telemetry.addData("Left Actuator Velo: ", leftActuator.getVelocity());
            telemetry.addData("Right Actuator Velo: ", rightActuator.getVelocity());
            telemetry.addData("Left Actuator Current: ", leftActuator.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Actuator Current: ", rightActuator.getCurrent(CurrentUnit.AMPS));

//            telemetry.addData("slide pos", slide.getCurrentPosition());
//            telemetry.addData("Intake Power", intake.getPower());
//            telemetry.addData("Intake Current", intake.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
