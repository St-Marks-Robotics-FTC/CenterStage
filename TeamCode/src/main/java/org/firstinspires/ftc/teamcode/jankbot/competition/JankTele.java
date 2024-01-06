package org.firstinspires.ftc.teamcode.jankbot.competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.jankbot.Jankbot;

@Config
@TeleOp
public class JankTele extends LinearOpMode {

    enum LinearStates {
        IDLE1,
        INTAKE,
        TILT,
        TRANSFER,

        IDLE2,
        EXTEND,

        IDLE3,
        SCORE,
        RETRACT
    }
    
    int slideLevel = 1;
    int turretLevel = 0;

    boolean hangReady = false;
    double loopTime = 0;

    Jankbot robot;
    GamepadEx pad1, pad2;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Jankbot(hardwareMap);
        pad1 = new GamepadEx(gamepad1);
        ToggleButtonReader hangToggle = new ToggleButtonReader(
                pad1, GamepadKeys.Button.DPAD_RIGHT
        );


        pad2 = new GamepadEx(gamepad2);
        ElapsedTime time = new ElapsedTime();
        
        
        
        
        // State Machine

        StateMachine machine = new StateMachineBuilder()
                .state(LinearStates.IDLE1)
                .onEnter( () -> {
                    robot.outtake.setV4Bar(0.3); // V4b Stow Position
                    robot.outtake.turretTransfer(); // Turret Vertical
                    robot.outtake.setSlides(0); // Retract Slide
                })
                .transition( () ->  gamepad1.right_trigger > 0.5 ) // Intake Button

                .state(LinearStates.INTAKE)
                .onEnter( () -> {
                    robot.intake.drop(); // Drop Intake
                    robot.intake.setIntake(0.8); // Spin Intake
                })
                .transition( () -> gamepad1.right_trigger < 0.5) // if let go or intake sensor


                .state(LinearStates.TILT)
                .onEnter( () -> {
                    robot.intake.setIntake(0); // Stop Intake
                    robot.intake.tiltUp(); // Intake tilt

                    robot.outtake.v4barTransfer(); // V4Bar down
                })
                .transitionTimed(0.75)
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.INTAKE) // Intake Again


                .state(LinearStates.TRANSFER)
                .onEnter( () -> {
                    robot.outtake.closeClaw(); // Claw Grab
                })
                .transitionTimed(0.5)
                .onExit( () -> {
                    robot.outtake.v4barStow(); // V4b Stow Position
                })
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.INTAKE) // Intake Again







                .state(LinearStates.IDLE2)
                .transition( () ->  gamepad1.y) // Outtake Button

                .state(LinearStates.EXTEND)
                .onEnter( () -> {
                    robot.outtake.slidesTo(slideLevel); // Extend Slide
                    robot.outtake.v4barScore(); // V4b Score Position
                    robot.outtake.turretRight(); // Spin Turret to horizontal
                })
                .loop( () -> {
                    if (pad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                        robot.outtake.turretLeft(); // Spin Turret Left
                    } else if (pad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        robot.outtake.turretRight(); // Spin Turret Right
                    }

                    robot.outtake.slidesTo(slideLevel); // Extend Slide
                })
                .transition( () ->  robot.outtake.getSlidePos() > 100) // Checks if slides are out






                .state(LinearStates.IDLE3)
                .loop( () -> {
                    if (pad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                        robot.outtake.turretLeft(); // Spin Turret Left
                    } else if (pad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        robot.outtake.turretRight(); // Spin Turret Right
                    }

                    robot.outtake.slidesTo(slideLevel); // Extend Slide
                })



                .transition( () ->  gamepad1.b) // Score Button
                .transition(() -> gamepad1.a,  // Retract Button
                        LinearStates.IDLE2,
                        () -> {
                            robot.outtake.v4barStow(); // V4b Stow Position
                            robot.outtake.turretTransfer(); // Turret Vertical
                            robot.outtake.retractSlides(); // Retract Slide
                        })


                .state(LinearStates.SCORE)
                .onEnter( () -> {
                    robot.outtake.openClaw(); // Open Claw
                })
                .transitionTimed(0.3)

                .state(LinearStates.RETRACT)
                .onEnter( () -> {
                    robot.outtake.v4barStow(); // V4b Stow Position
                    robot.outtake.turretTransfer(); // Turret Vertical
                    robot.outtake.retractSlides(); // Retract Slide
                })
                .transition( () ->  robot.outtake.getSlidePos() < 15) // Checks if slides are down



                .build();
        
        
        
        
        




        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double tranScaleFactor = gamepad1.left_bumper ? 0.4 : 1.0;
            double rotScaleFactor = gamepad1.left_bumper ? 0.3 : 1.0;
            robot.drive.setWeightedDrivePower(
                    new Pose2d(tranScaleFactor * y, tranScaleFactor * x, rotScaleFactor * 0.3 * rx)
            );








            machine.update();















            // Special Teams
            if (pad1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                robot.special.shootDrone();
            }

            if (hangToggle.getState()) {
                robot.special.hangReady();
                hangReady = true;
            } else if (hangReady){
                robot.special.hangClimb();
            }











            robot.drive.update();

            Pose2d poseEstimate = robot.drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());


            // in da loop
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;

            telemetry.update();
            pad1.readButtons();
            hangToggle.readValue();
            pad2.readButtons();

        }
    }
}