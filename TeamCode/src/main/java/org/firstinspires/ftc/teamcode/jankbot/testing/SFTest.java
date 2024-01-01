package org.firstinspires.ftc.teamcode.jankbot.testing;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.jankbot.Jankbot;

@Config
@TeleOp (group = "test")
public class SFTest extends LinearOpMode {

    Jankbot robot;
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

    int level = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Jankbot robot = new Jankbot(hardwareMap);
        GamepadEx pad1 = new GamepadEx(gamepad1);
        GamepadEx pad2 = new GamepadEx(gamepad2);


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
                    robot.intake.lock(); // Intake lock
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
                    robot.outtake.slidesTo(level); // Extend Slide
                    robot.outtake.v4barScore(); // V4b Score Position
                    robot.outtake.turretRight(); // Spin Turret to horizontal
                })
                .loop( () -> {
                    if (pad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                        robot.outtake.turretLeft(); // Spin Turret Left
                    } else if (pad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        robot.outtake.turretRight(); // Spin Turret Right
                    }

                    robot.outtake.slidesTo(level); // Extend Slide
                })
                .transition( () ->  robot.outtake.getSlidePos() > 100) // Checks if slides are out






                .state(LinearStates.IDLE3)
                .loop( () -> {
                    if (pad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                        robot.outtake.turretLeft(); // Spin Turret Left
                    } else if (pad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        robot.outtake.turretRight(); // Spin Turret Right
                    }

                    robot.outtake.slidesTo(level); // Extend Slide
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

        machine.start();

        while(opModeIsActive()) { //  loop
            machine.update();


            if (pad2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                level = Math.min(9, level+1);
            } else if (pad2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                level = Math.max(0, level-1);
            }


            pad1.readButtons();
            pad2.readButtons();

            telemetry.addData("State", machine.getState());
            telemetry.update();
        }
    }
}