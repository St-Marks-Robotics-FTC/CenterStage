package org.firstinspires.ftc.teamcode.jankbot.competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

import org.firstinspires.ftc.teamcode.jankbot.Jankbot;

import java.util.List;

@Config
@Disabled
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

        // Bulk Reading
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        robot = new Jankbot(hardwareMap);
        pad1 = new GamepadEx(gamepad1);
        ToggleButtonReader hangToggle = new ToggleButtonReader(
                pad1, GamepadKeys.Button.DPAD_RIGHT
        );


        pad2 = new GamepadEx(gamepad2);
        ElapsedTime time = new ElapsedTime();
        
        
        
        
        // State Machine

        StateMachine machine = new StateMachineBuilder()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                .state(LinearStates.IDLE1)                 // Driving to wing to pick up
                .onEnter( () -> {
                    robot.outtake.v4barTransfer(); // V4b ready for transfer
                    robot.outtake.turretTransfer(); // Turret Vertical
                    robot.outtake.retractSlides(); // Retract Slide
                })
                .transition( () ->  gamepad1.right_trigger > 0.5 ) // Intake Button

                .state(LinearStates.INTAKE)
                .onEnter( () -> {
                    robot.intake.tiltDown(); // Drop Intake
                    robot.intake.setIntake(0.8); // Spin Intake
                })
                .transition( () -> gamepad1.right_trigger < 0.5) // if let go or intake sensor


                .state(LinearStates.TILT)
                .onEnter( () -> {
                    robot.intake.setIntake(0); // Stop Intake
                    robot.intake.tiltUp(); // Intake tilts up
                })
                .transitionTimed(0.75)
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.INTAKE) // Intake Again if we missed


                .state(LinearStates.TRANSFER)
                .onEnter( () -> {
                    robot.outtake.closeBothClaws(); // Claw Grab
                })
                .transitionTimed(0.5)
                .onExit( () -> {
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
                        robot.outtake.manualSlides(gamepad2.left_stick_y);
                    } else {
                        robot.outtake.slidesToLevel(slideLevel); // Extend Slide to Level
                    }

                })
                .transition( () ->  gamepad1.b) // Score Button
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
                })
                .transition( () ->  robot.outtake.getSlidePos() < 15, LinearStates.IDLE1) // Checks if slides are down, goes back to IDLE1



                .build();
        
        
        
        
        




        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Will run one bulk read per cycle,
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }


            

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double tranScaleFactor = gamepad1.left_bumper ? 0.4 : 1.0;
            double rotScaleFactor = gamepad1.left_bumper ? 0.3 : 1.0;
            robot.drive.setWeightedDrivePower(
                    new Pose2d(tranScaleFactor * y, tranScaleFactor * x, rotScaleFactor * 0.3 * rx)
            );





            if (pad2.wasJustPressed(GamepadKeys.Button.DPAD_UP) || pad1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                slideLevel = Math.min(9, slideLevel + 1);
            } else if (pad2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) || pad1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                slideLevel = Math.max(0, slideLevel - 1);
            }

            if (pad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                turretLevel = Math.min(4, turretLevel+1);
            } else if (pad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                turretLevel = Math.max(-4, turretLevel-1);
            }


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