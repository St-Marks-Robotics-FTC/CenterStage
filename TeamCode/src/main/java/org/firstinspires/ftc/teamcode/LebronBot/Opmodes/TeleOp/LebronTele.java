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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.MecanumDrive;

import java.util.List;

@Config
@Disabled
@TeleOp
public class LebronTele extends LinearOpMode {

    enum LinearStates {
        IDLE1,
        INTAKE,
        SPIT,
        TILT,
        TRANSFER,
        STOW,

        IDLE2,
        EXTEND,

        IDLE3,
        SCORE,
        RETRACT
    }


    int slideLevel = 1;
    int turretLevel = 0;
    boolean manualSlides = false;

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
                    robot.outtake.v4barTransfer(); // V4b ready for transfer
                    robot.outtake.turretTransfer(); // Turret Vertical
                    robot.outtake.retractSlides(); // Retract Slide
                })
                .transition( () ->  gamepad1.right_bumper, LinearStates.TILT) // Manual Transfer if 1 pixel
                .transition( () ->  gamepad1.right_trigger > 0.5 ) // Intake Button Main one

                .state(LinearStates.INTAKE)
                .onEnter( () -> {
                    robot.intake.tiltDown(); // Drop Intake
                    robot.intake.setIntake(0.8); // Spin Intake
                })
                .transition( () -> (gamepad1.right_trigger < 0.5) && (!robot.intake.getPixel1() || !robot.intake.getPixel2()) , LinearStates.IDLE1) // if let go and not both pixels
                .transition( () -> robot.intake.getPixel1() && robot.intake.getPixel2())


                .state(LinearStates.SPIT)
                .onEnter( () -> {
                    robot.intake.setIntake(-0.08); // Spit Intake
                    robot.intake.tiltUp(); // Intake tilts up
                })
                .transitionTimed(0.2)

                .state(LinearStates.TILT)
                .onEnter( () -> {
                    robot.intake.setIntake(0.08); // suck in
                    robot.intake.tiltUp(); // Intake tilts up
                })
                .transitionTimed(0.5)
                .transition( () ->  robot.intake.isTiltUp()) // Tilt is up
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.INTAKE) // Intake Again if we missed


                .state(LinearStates.TRANSFER)
                .onEnter( () -> {
                    robot.intake.setIntake(0); // Stop Intake
                    robot.outtake.closeBothClaws(); // Claw Grab
                })
                .transitionTimed(0.5)
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.INTAKE) // Intake Again if we missed

                .state(LinearStates.STOW)
                .onEnter( () -> {
                    robot.outtake.v4barStow(); // V4b Stow Position
                })
                .onExit( () -> {
                    robot.intake.tiltStow(); // Intake Stow
                })
                .transitionTimed(0.25)




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



                .state(LinearStates.IDLE2)                                   // Have pixels in claw, driving back to backboard
                .transition( () ->  gamepad1.y) // Outtake Button

                .state(LinearStates.EXTEND)
                .onEnter( () -> {
                    robot.outtake.slidesToLevel(slideLevel); // Extend Slide
                    robot.outtake.v4barScore(); // V4b Score Position
                })
                .loop( () -> {
                    robot.outtake.turretTo(turretLevel); // Spin Turret
                    robot.outtake.slidesToLevel(slideLevel); // Extend Slide
                })
                .transition( () ->  robot.outtake.getSlidePos() > 100) // Checks if slides are out



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


                .state(LinearStates.IDLE3)                               // Slides out, ready to place
                .loop( () -> {
                    if (pad2.wasJustPressed(GamepadKeys.Button.X)) {
                        robot.outtake.openLeft(); // Open Left Claw
                    } else if (pad2.wasJustPressed(GamepadKeys.Button.B)) {
                        robot.outtake.openRight(); // Open Right Claw
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
                .transition( () ->  gamepad1.b) // Score Button
                .transition( () ->  robot.outtake.isClawOpen()) // if both sides were individually opened
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
                .transition( () ->  robot.outtake.getSlidePos() < 15, LinearStates.IDLE1) // Checks if slides are down, goes back to IDLE1


                .build();






        waitForStart();

        targetAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        robot.special.holdDrone();
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

            robot.drive.setMotorPowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);










            // Slide Level Adjustments
            if (pad2.wasJustPressed(GamepadKeys.Button.DPAD_UP) || pad1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                slideLevel = Math.min(9, slideLevel + 1);
            } else if (pad2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) || pad1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                slideLevel = Math.max(0, slideLevel - 1);
            }

            // Turret Angle Adjustments
            if (pad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                turretLevel = Math.min(4, turretLevel+1);
            } else if (pad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                turretLevel = Math.max(-4, turretLevel-1);
            }




            // Drone
            if (droneToggle.getState()) {
                robot.special.shootDrone();
            } else {
                robot.special.holdDrone();
            }











            // Telemetry
            telemetry.addData("State", machine.getState());

            telemetry.addData("Slide Level", slideLevel);
            telemetry.addData("Turret Pos", turretLevel);
            telemetry.addData("Manual Slides", manualSlides);




            machine.update();

            pad1.readButtons();
            droneToggle.readValue();
            pad2.readButtons();


            // in da loop
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;

            telemetry.update();

        }
    }
}