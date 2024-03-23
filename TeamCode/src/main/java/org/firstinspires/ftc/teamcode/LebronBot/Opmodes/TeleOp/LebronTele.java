package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.TeleOp;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.MecanumDrive;

import java.util.List;

@Config
//@Disabled
@TeleOp
public class LebronTele extends LinearOpMode {

    enum LinearStates {
        IDLE1,
        INTAKE,
        SUCK,
        TILT,
        DROP_OUTTAKE,
        TRANSFER,
        STOW,

        IDLE2,
        EXTEND,

        IDLE3,
        SCORE,
        RETRACT,

        INTAKE_AGAIN,
        STOWANGLE,
        ALIGN
    }


    int slideLevel = 1;
    int turretLevel = 0;
    boolean manualSlides = false;

    boolean leftClosed = true;
    boolean rightClosed = true;

    boolean hangReady = false;
    double loopTime = 0;

    LebronClass robot;
    GamepadEx pad1, pad2;


    // Heading Auto Align
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

        // Bulk Reading
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        robot = new LebronClass(hardwareMap);
        pad1 = new GamepadEx(gamepad1);
        ToggleButtonReader hangToggle = new ToggleButtonReader(
                pad1, GamepadKeys.Button.DPAD_RIGHT
        );
        ToggleButtonReader droneToggle = new ToggleButtonReader(
                pad1, GamepadKeys.Button.BACK
        );
        TriggerReader leftTrigger = new TriggerReader(
                pad1, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        TriggerReader rightTrigger = new TriggerReader(
                pad1, GamepadKeys.Trigger.RIGHT_TRIGGER
        );
        pad2 = new GamepadEx(gamepad2);


        // PID
        controller = new PIDController(p, i , d);

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);


        ElapsedTime time = new ElapsedTime();




        // MAIN State Machine
        StateMachine machine = new StateMachineBuilder()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                .state(LinearStates.IDLE1)                 // Driving to wing to pick up
                .onEnter( () -> { // Happens on Init as well
                    robot.intake.setIntake(0); // Stop Intake
                    robot.intake.tiltStow(); // Intake Stow

                    robot.outtake.openBothClaws(); // Claw Open
                    robot.outtake.v4barStow(); // V4b ready for transfer
                    robot.outtake.v4barAngleTransfer();
                    robot.outtake.turretTransfer(); // Turret Vertical
//                    robot.outtake.retractSlides(); // Retract Slide

                    robot.outtake.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.outtake.midSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.outtake.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                })
                .transition( () ->  gamepad1.right_bumper, LinearStates.TILT) // Manual Transfer if 1 pixel
                .transition( () ->  gamepad1.right_trigger > 0.5 ) // Intake Button Main one

                .state(LinearStates.INTAKE)
                .onEnter( () -> {
                    robot.intake.tiltDown(); // Drop Intake
                    robot.intake.setIntake(0.8); // Spin Intake
                    robot.outtake.openBothClaws(); // Claw Open
                    robot.outtake.turretTransfer();

                })
                .transition( () -> (gamepad1.right_trigger < 0.5)) // if let go and not both pixels
//                .transition( () -> robot.intake.getPixel1() && robot.intake.getPixel2())


                .state(LinearStates.SUCK)
                .onEnter( () -> {
                    robot.intake.setIntake(0.25); // keep Intaking
                    robot.intake.tiltUp(); // Intake tilts up
                    robot.outtake.turretTransfer();
                })
                .transitionTimed(0.5)

                .state(LinearStates.TILT)
                .onEnter( () -> {
                    robot.intake.setIntake(0); // suck in
                    robot.intake.tiltUp(); // Intake tilts up
                    robot.outtake.turretTransfer();
                })
                .transitionTimed(0.5)
//                .transition( () ->  robot.intake.isTiltUp()) // Tilt is up
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.IDLE1) // Intake Again if we missed

                .state(LinearStates.DROP_OUTTAKE)
                .onEnter( () -> {
                    robot.outtake.turretTransfer();
                    robot.intake.setIntake(0); // Stop Intake
                    robot.outtake.v4barTransfer();
                })
                .transitionTimed(0.75)

                .state(LinearStates.TRANSFER)
                .onEnter( () -> {
                    robot.intake.setIntake(0); // Stop Intake
                    robot.outtake.closeBothClaws(); // Claw Grab
                    robot.outtake.turretTransfer();
                })
                .transitionTimed(0.3)
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.IDLE1) // Intake Again if we missed

                .state(LinearStates.STOW)
                .onEnter( () -> {
                    robot.outtake.setV4Bar(robot.outtake.v4barStow); // V4b Stow Position
                    robot.outtake.turretTransfer();
                    //robot.intake.tiltStow();
                })
                .transitionTimed(0.5)
                .state(LinearStates.STOWANGLE)
                .onEnter( () -> {
                    robot.intake.tiltStow();
                    robot.outtake.v4barAngleStow();
                    robot.outtake.turretTransfer();

                })
                .transitionTimed(0.25)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



                .state(LinearStates.IDLE2)                                   // Have pixels in claw, driving back to backboard
                .loop( () -> {
                    robot.outtake.turretTransfer();
                })
                .transition( () ->  gamepad1.y) // Outtake Button
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.INTAKE_AGAIN) // Intake Again if we missed


                .state(LinearStates.EXTEND)
                .onEnter( () -> {
                    robot.outtake.slidesToLevel(slideLevel); // Extend Slide
                    robot.outtake.v4barScore(); // V4b Score Position
                })
                .loop( () -> {
                    robot.outtake.slidesToLevel(slideLevel); // Extend Slide
                })
                .transitionTimed(1)


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


                .state(LinearStates.IDLE3)                               // Slides out, ready to place
                .onEnter( () -> {
                    leftClosed = true;
                    rightClosed = true;
                })
                .loop( () -> {
                    if (leftTrigger.wasJustPressed()) {
                        leftClosed = !leftClosed;
                    }
                    if (rightTrigger.wasJustPressed()) {
                        rightClosed = !rightClosed;
                    }

                    if (leftClosed) {
                        robot.outtake.closeLeftMore();
                        leftClosed = true;
                    } else {
                        robot.outtake.openLeft();
                        leftClosed = false;
                    }
                    if (rightClosed) {
                        robot.outtake.closeRightMore();
                        rightClosed = true;
                    } else {
                        robot.outtake.openRight();
                        rightClosed = false;
                    }

                    robot.outtake.turretTo(turretLevel); // Spin Turret
                    // Manual Control
                    if (Math.abs(pad2.getLeftY()) >= 0.1) {
                        manualSlides = true;
                        robot.outtake.manualSlides(pad2.getLeftY());
                    } else if (!manualSlides) {
                        robot.outtake.slidesToLevel(slideLevel); // Extend Slide to Level
                    }

                })
                .onExit( () -> {
                        leftClosed=true;
                        rightClosed=true;
                })
                .transition( () ->  gamepad1.right_bumper && !leftClosed && !rightClosed) // Both open
//                .transition( () ->  robot.outtake.isClawOpen()) // if both sides were individually opened
                .transition(() -> gamepad1.a,  // Retract Button Failsafe
                        LinearStates.IDLE2,
                        () -> {
                            robot.outtake.v4barStow(); // V4b Stow Position
                            robot.outtake.turretTransfer(); // Turret Vertical
                            robot.outtake.retractSlides(); // Retract Slide
                        })



                .state(LinearStates.SCORE)
                .onEnter( () -> {
                    robot.outtake.openBothClaws(); // Open Claw
                })
                .transitionTimed(0.3)


                .state(LinearStates.RETRACT)
                .onEnter( () -> {
                    robot.outtake.v4barStow(); // V4b Stow Position
                    robot.outtake.turretTransfer(); // Turret Vertical
                    robot.outtake.retractSlides(); // Retract Slide

                    slideLevel = 1;
                    turretLevel = 0;
                    manualSlides = false;
                })
                .onExit( () -> {
                    robot.outtake.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.outtake.midSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.outtake.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                })
                .transitionTimed(0.75)
                .transition( () ->  robot.outtake.getSlidePos() < 25, LinearStates.IDLE1) // Checks if slides are down, goes back to IDLE1


                .state(LinearStates.INTAKE_AGAIN)
                .onEnter( () -> {
                    robot.outtake.setV4Bar(0.5); // V4b Stow Position
                    robot.outtake.v4barAngle.setPosition(0.7); // V4b Stow Position

                })
                .transitionTimed(0.5, LinearStates.IDLE1)
                .build();






        waitForStart();

        targetAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

//        robot.special.holdDrone();
        machine.start();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Will run one bulk read per cycle,
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }


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

            robot.drive.setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);










            // Slide Level Adjustments
            if (pad2.wasJustPressed(GamepadKeys.Button.DPAD_UP) || pad1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                slideLevel = Math.min(6, slideLevel + 1);
            } else if (pad2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) || pad1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                slideLevel = Math.max(0, slideLevel - 1);
            }

            // Turret Angle Adjustments
            if (pad1.wasJustPressed(GamepadKeys.Button.X)) {
                turretLevel = Math.min(3, turretLevel+1);
            } else if (pad1.wasJustPressed(GamepadKeys.Button.B)) {
                turretLevel = Math.max(-3, turretLevel-1);
            }



//
//            // Drone
//            if (droneToggle.getState()) {
//                robot.special.shootDrone();
//            } else {
//                robot.special.holdDrone();
//            }



            if (robot.outtake.leftSlide.getVelocity() < 1 && robot.outtake.leftSlide.getCurrent(CurrentUnit.AMPS) > 1 && robot.outtake.leftSlide.getCurrentPosition() < 25) {
                robot.outtake.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (robot.outtake.midSlide.getVelocity() < 1 && robot.outtake.midSlide.getCurrent(CurrentUnit.AMPS) > 1 && robot.outtake.midSlide.getCurrentPosition() < 25 ) {
                robot.outtake.midSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (robot.outtake.rightSlide.getVelocity() < 1 && robot.outtake.rightSlide.getCurrent(CurrentUnit.AMPS) > 1 && robot.outtake.rightSlide.getCurrentPosition() < 25 ) {
                robot.outtake.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }









            // Telemetry
            telemetry.addData("State", machine.getState());

            telemetry.addData("Slide Level", slideLevel);
            telemetry.addData("Turret Pos", turretLevel);
            telemetry.addData("Manual Slides", manualSlides);


            telemetry.addData("Left Slide Position", robot.outtake.leftSlide.getCurrentPosition());
            telemetry.addData("Mid Slide Position", robot.outtake.midSlide.getCurrentPosition());
            telemetry.addData("Right Slide Position", robot.outtake.rightSlide.getCurrentPosition());

            telemetry.addData("Left Slide Current", robot.outtake.leftSlide.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Mid Slide Current", robot.outtake.midSlide.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Slide Current", robot.outtake.rightSlide.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Left V4B Pos: ", robot.outtake.v4barLeft.getPosition());
            telemetry.addData("Right V4B Pos: ", robot.outtake.v4barRight.getPosition());

//            telemetry.addData("Servo Current", robot.outtake.v4barLeft.get)




            machine.update();

            pad1.readButtons();
            droneToggle.readValue();
            leftTrigger.readValue();
            rightTrigger.readValue();
            pad2.readButtons();


            // in da loop
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;

            telemetry.update();

        }
    }
}