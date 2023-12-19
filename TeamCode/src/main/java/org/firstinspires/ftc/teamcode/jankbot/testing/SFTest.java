package org.firstinspires.ftc.teamcode.jankbot.testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.jankbot.Jankbot;

@TeleOp
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

    @Override
    public void runOpMode() throws InterruptedException {
        Jankbot robot = new Jankbot(hardwareMap);


        StateMachine machine = new StateMachineBuilder()
                .state(LinearStates.IDLE1)
                .transition( () ->  gamepad1.right_trigger > 0.5 ) // Intake Button

                .state(LinearStates.INTAKE)
                .onEnter( () -> {
//                    robot.intake.dropdown(); // Drop Intake
                    robot.intake.setIntake(0.8); // Spin Intake
                })
                .transition( () -> gamepad1.right_trigger < 0.5) // if let go or intake sensor


                .state(LinearStates.TILT)
                .onEnter( () -> {
                    robot.intake.setIntake(0); // Stop Intake
//                    robot.intake.; // Intake lock
//                    robot.intake.; // Intake tilt
                    robot.outtake.setV4Bar(0); // V4Bar down
                })
                .transitionTimed(0.75)
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.INTAKE) // Intake Again


                .state(LinearStates.TRANSFER)
                .onEnter( () -> {
//                    robot.outtake.(); // Claw Grab
                })
                .transitionTimed(0.5)
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.INTAKE) // Intake Again







                .state(LinearStates.IDLE2)
                .onEnter( () -> {
                    robot.outtake.setV4Bar(0.3); // V4b Stow Position
                })
                .transition( () ->  gamepad1.y) // Outtake Button

                .state(LinearStates.EXTEND)
                .onEnter( () -> {
                    robot.outtake.setSlide(800); // Extend Slide
                    robot.outtake.setV4Bar(0.5); // V4b Score Position
//                    robot.outtake.rotate(); // Spin Turret to horizontal
                })
                .transition( () ->  Math.abs(robot.outtake.getSlidePos() - 800) < 15) // Checks if slides are in position






                .state(LinearStates.IDLE3)
                .transition( () ->  gamepad1.b) // Score Button

                .state(LinearStates.SCORE)
                .onEnter( () -> {
                    robot.outtake.openClaw(); // Open Claw
                })
                .transitionTimed(0.5)

                .state(LinearStates.RETRACT)
                .onEnter( () -> {
                    robot.outtake.setV4Bar(0.3); // V4b Stow Position
//                    robot.outtake.rotate(0); // Turret Vertical
                    robot.outtake.setSlide(0); // Retract Slide
                })



                .build();

        waitForStart();

        machine.start();

        while(opModeIsActive()) { // autonomous loop
            machine.update();
        }
    }
}