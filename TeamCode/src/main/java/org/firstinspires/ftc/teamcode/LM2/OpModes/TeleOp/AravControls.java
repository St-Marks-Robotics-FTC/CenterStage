package org.firstinspires.ftc.teamcode.LM2.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.LM2.LM2class;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.MecanumDrive;

import java.util.List;

@Config
@TeleOp
public class AravControls extends LinearOpMode {

    double loopTime = 0;

    enum LinearStates {
        DOWN,
        UP
    }
    enum SpecialStates {
        HANG,
        PUSH
    }


    // Arm
    public static int armDown = 0;
    public static int armUp = 350;
    public static int armUp2 = 405; //CHANGE
    public static int armUp3 = 490; //CHANGE
    public static int[] armPos = {armDown,armUp2,armUp3};
    public static int level = 0;

    public static boolean dpadupPressed = false;
    public static int[] hangPos = {970,2200}; //CHANGE

    // Drone
    public static boolean droneLaunch = false;

    // Claw
    public static boolean closed = false;
    public static boolean leftClosed = false;
    public static boolean rightClosed = false;



    public static LM2class robot;
    MecanumDrive drive;
    public static GamepadEx pad1;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);


        robot = new LM2class(hardwareMap);
        drive = new MecanumDrive(hardwareMap);
        pad1 = new GamepadEx(gamepad1);
        TriggerReader leftTrigger = new TriggerReader(
                pad1, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        TriggerReader rightTrigger = new TriggerReader(
                pad1, GamepadKeys.Trigger.RIGHT_TRIGGER
        );

        ElapsedTime time = new ElapsedTime();



        // MAIN State Machine
        StateMachine machine = new StateMachineBuilder()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                .state(LinearStates.DOWN)                 // Driving to wing to pick up
                .onEnter( () -> { // Happens on Init as well
                    level = 0;
                    robot.setArm(armPos[level]);
                    robot.openClaw();
                    leftClosed = false;
                    rightClosed = false;
                })
                .loop( () -> {
                    if (gamepad1.left_trigger >= 0.3) {
                        robot.openClaw();
                        leftClosed = false;
                        rightClosed = false;
                    } else if (gamepad1.right_trigger >= 0.3) {
                        robot.closeClaw();
                        leftClosed = true;
                        rightClosed = true;
                    }

                    if (robot.detectLeft()) {
                        if (!leftClosed)
                            gamepad1.rumble(0.25, 0, 500);
                        robot.closeLeft();
                        leftClosed = true;
                    }
                    if (robot.detectRight()) {
                        if (!rightClosed)
                            gamepad1.rumble(0.0, 0.25, 500);
                        robot.closeRight();
                        rightClosed = true;
                    }
                })
                .transition( () ->  pad1.wasJustPressed(GamepadKeys.Button.B), SpecialStates.PUSH) // Let go on ground
                .transition( () ->  pad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) ) // Raise arm
                .transition( () ->  pad1.wasJustPressed(GamepadKeys.Button.DPAD_UP), SpecialStates.HANG) // Hang





                .state(LinearStates.UP)
                .onEnter( () -> {
                    level = 1;
                    robot.setArm(armPos[level]);
                })
                .loop( () -> {
                    if (leftTrigger.wasJustPressed()) {
                        leftClosed = !leftClosed;
                    }
                    if (rightTrigger.wasJustPressed()) {
                        rightClosed = !rightClosed;
                    }

                    if (leftClosed) {
                        robot.closeLeft();
                        leftClosed = true;
                    } else {
                        robot.scoreLeft();
                        leftClosed = false;
                    }
                    if (rightClosed) {
                        robot.closeRight();
                        rightClosed = true;
                    } else {
                        robot.scoreRight();
                        rightClosed = false;
                    }

                    if (gamepad1.y) {
                        robot.setArm(robot.arm.getCurrentPosition()+15);
                    } else if(gamepad1.a){
                        robot.setArm(robot.arm.getCurrentPosition()-20);
                    }

                    if (pad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                        if(level == 2){
                            level = 1;
                            robot.setArm(armPos[level]);
                        } else if (level == 1){
                            level = 2;
                            robot.setArm(armPos[level]);
                        }
                    }

                })

                .transition( () ->  pad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && !leftClosed && !rightClosed, LinearStates.DOWN) // right bumper and both sides open
                .transition( () ->  pad1.wasJustPressed(GamepadKeys.Button.X), LinearStates.DOWN) // Drop arm to ground with pixels in claw
                .transition( () ->  pad1.wasJustPressed(GamepadKeys.Button.DPAD_UP), SpecialStates.HANG) // Hang


                .state(SpecialStates.HANG)
                .onEnter( () -> {
                    robot.setArm(hangPos[0]);
                    dpadupPressed = true;
                })
                .loop( () -> {
                    // Hang
                    if(pad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                        if(!dpadupPressed){
                            robot.setArm(hangPos[0]);
                            dpadupPressed = true;
                        } else {
                            robot.hang(hangPos[1]);
                            dpadupPressed=false;
                        }
                    }
                })
                .transition( () ->  rightTrigger.wasJustPressed(), LinearStates.DOWN) // Drop Arm


                .state(SpecialStates.PUSH)
                .onEnter( () -> {
                    robot.openClaw();
                    leftClosed = false;
                    rightClosed = false;
                })
                .transitionTimed(0.75, LinearStates.DOWN)


                .build();

        dpadupPressed = false; // So arm doesnt flip all the way w
        droneLaunch = false;
        robot.openClaw();
        robot.holdDrone();
        waitForStart();
        machine.start();
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Will run one bulk read per cycle,
            // even as frontLeftMotor.getCurrentPosition() is called twice
            // because the caches are being handled manually and cleared
            // once a loop
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            double tranScaleFactor = gamepad1.left_bumper ? 0.4 : 1.0;
            double rotScaleFactor = gamepad1.left_bumper ? 0.4 : 0.9;

            final double DEADZONE = 0.075;

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Apply deadzones
            if (Math.abs(y) < DEADZONE) {
                y = 0.0;
            }
            if (Math.abs(x) < DEADZONE) {
                x = 0.0;
            }
            if (Math.abs(rx) < DEADZONE) {
                rx = 0.0;
            }

            // Apply scaling factors
            y *= tranScaleFactor;
            x *= tranScaleFactor;
            rx *= rotScaleFactor;



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



            machine.update();


            // Drone
            if (pad1.wasJustPressed(GamepadKeys.Button.BACK)) { // left of touchpad
                droneLaunch = !droneLaunch;
            }
            if (droneLaunch) {
                robot.shootDrone();
            } else {
                robot.holdDrone();
            }

            // Claw
            if (!rightClosed && !leftClosed) closed = false; // if both are open, claw is open
            else if (rightClosed && leftClosed) closed = true; // if both are closed, claw is closed




            // Hang
//            if (pad1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//                machine.setState(LinearStates.HANG);
//            }






            pad1.readButtons();
            leftTrigger.readValue();
            rightTrigger.readValue();


            telemetry.addData("Left Y", gamepad1.left_stick_y);
            telemetry.addData("Left X", gamepad1.left_stick_x);
            telemetry.addData("Right X", gamepad1.right_stick_x);

            telemetry.addData("State", machine.getState());
            telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
            telemetry.addData("Arm Target", robot.arm.getTargetPosition());
            telemetry.addData("Arm Power", robot.arm.getPower());
            telemetry.addData("Arm Current", robot.arm.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Claw Closed", closed);
            telemetry.addData("Left Claw closed", leftClosed);
            telemetry.addData("Right Claw closed", rightClosed);

            // in da loop
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();



        }
    }
}