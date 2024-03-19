package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.TeleOp;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;


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
        CHECK,
        TILT,
        TRANSFER,

        IDLE2,
        EXTEND,

        IDLE3,
        SCORE,
        RETRACT
    }

    enum JamStates {
        MAIN,
        EJECT,
        INTAKE
    }

    int slideLevel = 1;
    int turretLevel = 0;
    boolean manualSlides = false;

    boolean hangReady = false;
    double loopTime = 0;

    LebronClass robot;
    GamepadEx pad1, pad2;

    private PIDFController headingController = new PIDFController(MecanumDrive.HEADING_PID);
    double imuSetpoint = 180;



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
//        ToggleButtonReader headingToggle = new ToggleButtonReader(
//                pad1, GamepadKeys.Button.RIGHT_STICK_BUTTON
//        );


        pad2 = new GamepadEx(gamepad2);


        ElapsedTime time = new ElapsedTime();




        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
//        robot.drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);



        // JAM State Machine
        StateMachine jamMachine = new StateMachineBuilder()
                .state(JamStates.EJECT)
                .onEnter( () -> {
                    robot.intake.setIntake(-0.15);
                    robot.intake.tiltDown();
                })
                .transitionTimed(.25)


                .state(JamStates.INTAKE)
                .onEnter( () -> {
                    robot.intake.setIntake(0.8);
                    robot.intake.tiltUp();
                })
                .transitionTimed(.3)

                .build();



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



                .state(LinearStates.CHECK) // Accounts for delay between distance sensor and touch sensors
                .transition( () ->  robot.intake.is2Aligned(), () -> gamepad1.rumble(500))
                .transitionTimed(0.20)


                .state(LinearStates.TILT)
                .onEnter( () -> {
                    robot.intake.setIntake(0); // Stop Intake
                    robot.intake.tiltUp(); // Intake tilts up
                })
                .transition( () ->  robot.intake.is2NotAligned(), JamStates.MAIN) // JAM failsafe
                .transitionTimed(0.75)
                .transition( () ->  robot.intake.isTiltUp()) // Tilt is up
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.INTAKE) // Intake Again if we missed


                .state(LinearStates.TRANSFER)
                .onEnter( () -> {
                    robot.outtake.closeBothClaws(); // Claw Grab
                })
                .transitionTimed(0.5)
                .onExit( () -> {
                    robot.intake.tiltStow(); // Intake Stow
                    robot.outtake.v4barStow(); // V4b Stow Position
                })
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.INTAKE) // Intake Again if we missed



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


                // NESTED STATE MACHINE
                .state(JamStates.MAIN)
                .onEnter(jamMachine::start) // Starting the machine
                .loop(jamMachine::update) // Updating the machine in the loop
                .onExit( () -> {
                    jamMachine.reset(); // Stopping the machine when we exit the state
                    jamMachine.stop(); // Stopping the machine when we exit the state
                })

                .transition( () -> !jamMachine.isRunning(), LinearStates.TILT) // Transition when the transfer is done


                .build();






        waitForStart();

        robot.special.holdDrone();
        machine.start();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Will run one bulk read per cycle,
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }


            Pose2d poseEstimate = robot.drive.getPoseEstimate();

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x * 0.8;

            double tranScaleFactor = gamepad1.left_bumper ? 0.4 : 1.0;
            double rotScaleFactor = gamepad1.left_bumper ? 0.3 : 1.0;
            if (gamepad1.left_trigger >= 0.5) {
                headingController.setTargetPosition(Math.toRadians(imuSetpoint));

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = clamp(headingController.update(poseEstimate.getHeading()), -0.3, 0.3);

                robot.drive.setWeightedDrivePower(
                        new Pose2d(tranScaleFactor * y, tranScaleFactor * x, headingInput)
                );

            } else {
                robot.drive.setWeightedDrivePower(
                        new Pose2d(tranScaleFactor * y, tranScaleFactor * x, rotScaleFactor * rx)
                );
            }

            // Heading PID Adjustments
            if (pad2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) || pad1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                imuSetpoint += 2.5; // degrees
            } else if (pad2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) || pad1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                imuSetpoint -= 2.5; // degrees
            }







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




            // Special Teams
            if (pad1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                robot.special.shootDrone();
            }

            if (hangToggle.getState()) { // Dpad Right
                robot.special.hangReady();
                hangReady = true;
            } else if (hangReady){
                robot.special.hangClimb();
            }










            // Telemetry
            telemetry.addData("State", machine.getState());
            telemetry.addData("Jam State", jamMachine.getState());

            telemetry.addData("Slide Level", slideLevel);
            telemetry.addData("Turret Pos", turretLevel);
            telemetry.addData("Manual Slides", manualSlides);
            telemetry.addData("Heading Setpoint", imuSetpoint);


            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());


            // in da loop
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;

            robot.drive.update();
            machine.update();

            pad1.readButtons();
            hangToggle.readValue();
//            headingToggle.readValue();
            pad2.readButtons();

            telemetry.update();

        }
    }
}