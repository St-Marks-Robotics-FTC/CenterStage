package org.firstinspires.ftc.teamcode.LM2.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.checkerframework.checker.i18nformatter.qual.I18nFormat;
import org.firstinspires.ftc.teamcode.Fallback.Opmodes.TeleOp.FallbackTele;
import org.firstinspires.ftc.teamcode.LM2.LM2class;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.MecanumDrive;

@Config
@TeleOp
public class AravControls extends LinearOpMode {

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
        TriggerReader leftReader = new TriggerReader(
                pad1, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        TriggerReader rightReader = new TriggerReader(
                pad1, GamepadKeys.Trigger.RIGHT_TRIGGER
        );

        ElapsedTime time = new ElapsedTime();



        // MAIN State Machine
        StateMachine machine = new StateMachineBuilder()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                .state("DOWN")                 // Driving to wing to pick up
                .onEnter( () -> { // Happens on Init as well
                    level = 0;
                    robot.setArm(armPos[level]);
                    robot.openClaw();
                })
                .loop( () -> {
                    if (gamepad1.left_trigger >= 0.3) {
                        robot.openClaw();
                    } else if (gamepad1.right_trigger >= 0.3) {
                        robot.closeClaw();
                        leftClosed = true;
                        rightClosed = true;
                    }

                    if (robot.detectLeft()) {
                        robot.closeLeft();
                    }
                    if (robot.detectRight()) {
                        robot.closeRight();
                    }
                })
                .transition( () ->  pad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) ) // Raise arm





                .state("UP")
                .onEnter( () -> {
                    level = 1;
                    robot.setArm(armPos[level]);
                })
                .loop( () -> {
                    if (leftReader.wasJustPressed()) {
                        leftClosed = !leftClosed;
                    }
                    if (rightReader.wasJustPressed()) {
                        rightClosed = !rightClosed;
                    }

                    if (leftClosed) {
                        robot.scoreLeft();
                        leftClosed = true;
                    } else {
                        robot.openLeft();
                        leftClosed = false;
                    }
                    if (rightClosed) {
                        robot.scoreRight();
                        rightClosed = true;
                    } else {
                        robot.openRight();
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

                .transition( () ->  pad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && !leftClosed && !rightClosed, "DOWN") // right bumper and both sides open

                .build();















        dpadupPressed = false; // So arm doesnt flip all the way w
        droneLaunch = false;
        robot.openClaw();
        robot.closeDrone();
        waitForStart();
        machine.start();
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



            machine.update();


            // Drone
            if (pad1.wasJustPressed(GamepadKeys.Button.BACK)) {
                droneLaunch = !droneLaunch;
            }
            if (droneLaunch) {
                robot.openDrone();
            } else {
                robot.closeDrone();
            }

            // Claw
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
            leftReader.readValue();
            rightReader.readValue();


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