//package org.firstinspires.ftc.teamcode.jankbot.competition.auto;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.sfdev.assembly.state.StateMachine;
//import com.sfdev.assembly.state.StateMachineBuilder;
//
//import org.firstinspires.ftc.teamcode.jankbot.Jankbot;
//import org.firstinspires.ftc.teamcode.jankbot.competition.JankTele;
//
//@Config
//@Disabled
//@Autonomous (group = "Red", preselectTeleOp = "JankTele")
//public class RedCycle extends LinearOpMode {
//
//    enum LinearStates {
//        INTAKE,
//        TILT,
//        TRANFER,
//        EXTEND
//    }
//
//    Jankbot robot;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        robot = new Jankbot(hardwareMap);
//
//        // MAIN State Machine
//        StateMachine machine = new StateMachineBuilder()
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                .state(LinearStates.INTAKE)                 // Driving to wing to pick up
//                .onEnter( () -> { // Happens on Init as well
//                    robot.intake.setIntake(0.3); // Stop Intake
//                })
//                .transitionTimed(0.5)
//                .state(JankTele.LinearStates.INTAKE)
//                .onEnter( () -> {
//                    robot.intake.tiltDown(); // Drop Intake
//                    robot.intake.setIntake(0.8); // Spin Intake
//                })
//                .transition( () -> (gamepad1.right_trigger < 0.5) && (!robot.intake.getPixel1() || !robot.intake.getPixel2()) , JankTele.LinearStates.IDLE1) // if let go and not both pixels
//                .transition( () -> robot.intake.getPixel1() && robot.intake.getPixel2())
//
//
//
//                .state(JankTele.LinearStates.CHECK) // Accounts for delay between distance sensor and touch sensors
//                .transition( () ->  robot.intake.is2Aligned(), () -> gamepad1.rumble(500))
//                .transitionTimed(0.20)
//
//
//                .state(JankTele.LinearStates.TILT)
//                .onEnter( () -> {
//                    robot.intake.setIntake(0); // Stop Intake
//                    robot.intake.tiltUp(); // Intake tilts up
//                })
//                .transition( () ->  robot.intake.is2NotAligned(), JankTele.JamStates.MAIN) // JAM failsafe
//                .transitionTimed(0.75)
//                .transition( () ->  robot.intake.isTiltUp()) // Tilt is up
//                .transition( () ->  gamepad1.right_trigger > 0.5 , JankTele.LinearStates.INTAKE) // Intake Again if we missed
//
//
//                .state(JankTele.LinearStates.TRANSFER)
//                .onEnter( () -> {
//                    robot.outtake.closeBothClaws(); // Claw Grab
//                })
//                .transitionTimed(0.5)
//                .onExit( () -> {
//                    robot.intake.tiltStow(); // Intake Stow
//                    robot.outtake.v4barStow(); // V4b Stow Position
//                })
//                .transition( () ->  gamepad1.right_trigger > 0.5 , JankTele.LinearStates.INTAKE) // Intake Again if we missed
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//                .state(JankTele.LinearStates.IDLE2)                                   // Have pixels in claw, driving back to backboard
//                .transition( () ->  gamepad1.y) // Outtake Button
//
//                .state(JankTele.LinearStates.EXTEND)
//                .onEnter( () -> {
//                    robot.outtake.slidesToLevel(slideLevel); // Extend Slide
//                    robot.outtake.v4barScore(); // V4b Score Position
//                })
//                .loop( () -> {
//                    robot.outtake.turretTo(turretLevel); // Spin Turret
//                    robot.outtake.slidesToLevel(slideLevel); // Extend Slide
//                })
//                .transition( () ->  robot.outtake.getSlidePos() > 100) // Checks if slides are out
//
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//                .state(JankTele.LinearStates.IDLE3)                               // Slides out, ready to place
//                .loop( () -> {
//                    if (pad2.wasJustPressed(GamepadKeys.Button.X)) {
//                        robot.outtake.openLeft(); // Open Left Claw
//                    } else if (pad2.wasJustPressed(GamepadKeys.Button.B)) {
//                        robot.outtake.openRight(); // Open Right Claw
//                    }
//
//                    robot.outtake.turretTo(turretLevel); // Spin Turret
//                    // Manual Control
//                    if (Math.abs(pad2.getLeftY()) >= 0.1) {
//                        manualSlides = true;
//                        robot.outtake.manualSlides(pad2.getLeftY());
//                    } else if (!manualSlides) {
//                        robot.outtake.slidesToLevel(slideLevel); // Extend Slide to Level
//                    }
//
//                })
//                .transition( () ->  gamepad1.b) // Score Button
//                .transition( () ->  robot.outtake.isClawOpen()) // if both sides were individually opened
//                .transition(() -> gamepad1.a,  // Retract Button Failsafe
//                        JankTele.LinearStates.IDLE2,
//                        () -> {
//                            robot.outtake.v4barStow(); // V4b Stow Position
//                            robot.outtake.turretTransfer(); // Turret Vertical
//                            robot.outtake.retractSlides(); // Retract Slide
//                        })
//
//
//
//                .state(JankTele.LinearStates.SCORE)
//                .onEnter( () -> {
//                    robot.outtake.openBothClaws(); // Open Claw
//                })
//                .transitionTimed(0.3)
//
//
//                .state(JankTele.LinearStates.RETRACT)
//                .onEnter( () -> {
//                    robot.outtake.v4barStow(); // V4b Stow Position
//                    robot.outtake.turretTransfer(); // Turret Vertical
//                    robot.outtake.retractSlides(); // Retract Slide
//
//                    slideLevel = 1;
//                    turretLevel = 0;
//                    manualSlides = false;
//                })
//                .transition( () ->  robot.outtake.getSlidePos() < 15, JankTele.LinearStates.IDLE1) // Checks if slides are down, goes back to IDLE1
//
//
//                // NESTED STATE MACHINE
//                .state(JankTele.JamStates.MAIN)
//                .onEnter(jamMachine::start) // Starting the machine
//                .loop(jamMachine::update) // Updating the machine in the loop
//                .onExit( () -> {
//                    jamMachine.reset(); // Stopping the machine when we exit the state
//                    jamMachine.stop(); // Stopping the machine when we exit the state
//                })
//
//                .transition( () -> !jamMachine.isRunning(), JankTele.LinearStates.TILT) // Transition when the transfer is done
//
//
//                .build();
//
//        waitForStart();
//
//
//
//    }
//}
