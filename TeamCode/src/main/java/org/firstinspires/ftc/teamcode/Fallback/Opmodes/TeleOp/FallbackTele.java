package org.firstinspires.ftc.teamcode.Fallback.Opmodes.TeleOp;

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

import org.firstinspires.ftc.teamcode.Fallback.FallbackClass;
import org.firstinspires.ftc.teamcode.jankbot.Jankbot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;

import java.util.List;

@Config
@Disabled
@TeleOp
public class FallbackTele extends LinearOpMode {

    enum LinearStates {
        IDLE,
        DOWN,
        GRAB,
        STOW,
        EXTEND,
        SCORE,
        RETRACT
    }

    
    int slideLevel = 1;
    boolean manualSlides = false;

    boolean hangReady = false;
    double loopTime = 0;

    FallbackClass robot;
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


        robot = new FallbackClass(hardwareMap);
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

        


        // MAIN State Machine
        StateMachine machine = new StateMachineBuilder()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                .state(LinearStates.IDLE)                 // Driving to wing to pick up
                .onEnter( () -> { // Happens on Init as well
                    robot.openClaw(); // Claw Open
                    robot.v4barStow(); // V4b ready for transfer
                    robot.retractSlides(); // Retract Slide
                })
                .transition( () ->  gamepad1.right_trigger > 0.5 ) // Intake Button Main one

                .state(LinearStates.DOWN)
                .onEnter( () -> {
                    robot.v4barPickup(); // V4b Transfer Position
                })
                .transition( () ->  gamepad1.right_trigger <= 0.5 ) // Intake Button Main one

                .state(LinearStates.GRAB)
                .onEnter( () -> {
                    robot.closeClaw(); // Claw Grab
                })
                .transitionTimed(0.25)
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.DOWN) // Intake Again if we missed

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                .state(LinearStates.STOW)
                .onEnter( () -> {
                    robot.v4barStow(); // V4b Stow Position
                })
                .transition( () ->  gamepad1.y ) // outtake Button Main one

                .state(LinearStates.EXTEND)
                .onEnter( () -> {
                    robot.slidesToLevel(slideLevel); // Extend Slide
                    robot.v4barScore(); // V4b Score Position
                })
                .loop( () -> {
                    robot.slidesToLevel(slideLevel); // Extend Slide
                })
                .transition( () ->  robot.getSlidePos() > 100) // Checks if slides are out


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


                .state(LinearStates.SCORE)
                .onEnter( () -> {
                    robot.openClaw(); // Open Claw
                })
                .transitionTimed(0.3)


                .state(LinearStates.RETRACT)
                .onEnter( () -> {
                    robot.v4barStow(); // V4b Stow Position
                    robot.retractSlides(); // Retract Slide

                    slideLevel = 1;
                    manualSlides = false;
                })
                .transition( () ->  robot.getSlidePos() < 15, LinearStates.IDLE) // Checks if slides are down, goes back to IDLE1


                .build();






        waitForStart();

        robot.droneHold();
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



            // Special Teams
            if (pad1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                robot.droneRelease();
            }

//            if (hangToggle.getState()) { // Dpad Right
//                robot.special.hangReady();
//                hangReady = true;
//            } else if (hangReady){
//                robot.special.hangClimb();
//            }










            // Telemetry
            telemetry.addData("State", machine.getState());

            telemetry.addData("Slide Level", slideLevel);
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