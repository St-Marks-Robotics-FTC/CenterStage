package org.firstinspires.ftc.teamcode.LebronBot;

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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(group = "Testing")
public class ChassisTest extends LinearOpMode {

    public enum LinearStates {
        IDLE,
        DOWN,
        UP,
        OUTTAKE
    }

    boolean up = false;
    String pivot = "ground";

    GamepadEx pad1;
    TriggerReader leftTrigger;
    TriggerReader rightTrigger;

    DistanceSensor leftDist;
    DistanceSensor rightDist;
    private double maxPow = 1.0;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    DcMotorEx intake;
    DcMotor slide;

    Servo intakeAngle;

    @Override
    public void runOpMode() throws InterruptedException {
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

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        slide = hardwareMap.dcMotor.get("slide");

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeAngle = hardwareMap.servo.get("pivot");

        pad1 = new GamepadEx(gamepad1);
        leftTrigger = new TriggerReader(
                pad1, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        rightTrigger = new TriggerReader(
                pad1, GamepadKeys.Trigger.RIGHT_TRIGGER
        );

        // MAIN State Machine
        StateMachine machine = new StateMachineBuilder()
                .state(LinearStates.IDLE)                 // Driving to wing to pick up
                .onEnter(() -> { // Happens on Init as well
                    intake.setPower(0);
                    intakeAngle.setPosition(0.45); // up
                })
                .transition(() -> gamepad1.right_trigger >= 0.05) // Drop intake down
                .transition(() -> gamepad1.left_trigger >= 0.05, LinearStates.OUTTAKE) // Drop intake down

                .state(LinearStates.DOWN)
                .loop(() -> {
                    intake.setPower(1 * (gamepad1.right_trigger));
                    if (pivot.equals("ground")) {
                        intakeAngle.setPosition(0.8);
                    } else if (pivot.equals("stack")) {
                        intakeAngle.setPosition(0.71);
                    }
                })
                .transition(() -> gamepad1.right_trigger < 0.05) // Picks up when trigger let go
                .state(LinearStates.UP)
                .onEnter(() -> {
                    intake.setPower(0.75);
                    intakeAngle.setPosition(0.45); // up
                })
                .transitionTimed(1, LinearStates.IDLE)


                .state(LinearStates.OUTTAKE)
                .onEnter(() -> {
                    intake.setPower(-0.5);
                    intakeAngle.setPosition(0.8);
                })
                .transition(() -> gamepad1.left_trigger < 0.05, LinearStates.IDLE) // Picks up when trigger let go
                .build();






        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        machine.start();

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

            frontLeftMotor.setPower(maxPow * frontLeftPower);
            backLeftMotor.setPower(maxPow * backLeftPower);
            frontRightMotor.setPower(maxPow * frontRightPower);
            backRightMotor.setPower(maxPow * backRightPower);

            if (pad1.wasJustPressed(GamepadKeys.Button.A)) {
                pivot = "ground";
            } else if (pad1.wasJustPressed(GamepadKeys.Button.B)) {
                pivot = "stack";
            }

            machine.update();

            if (pad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                up = !up;
            }
            if (up) {
                slide.setTargetPosition(485);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
            } else {
                slide.setTargetPosition(5);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
            }

            pad1.readButtons();
            leftTrigger.readValue();
            rightTrigger.readValue();

            telemetry.addData("slide pos", slide.getCurrentPosition());
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Intake Current", intake.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
