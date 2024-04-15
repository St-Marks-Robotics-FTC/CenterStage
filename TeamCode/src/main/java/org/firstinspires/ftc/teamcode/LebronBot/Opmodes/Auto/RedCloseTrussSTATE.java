package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.Auto;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.LM2.Roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;
import org.firstinspires.ftc.teamcode.LebronBot.Opmodes.Testing.KALMAN;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.LebronBot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.AprilTagRelocalize;
import org.firstinspires.ftc.teamcode.Vision.Prop.RedFarPropThreshold;
import org.firstinspires.ftc.teamcode.Vision.Prop.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Config
@Autonomous
public class RedCloseTrussSTATE extends  LinearOpMode{
    enum LinearStates {
        PURPLE,
        PURPLEPAUSE,
        PAUSE,
        PURPLE2STACK,
        IDLE1,
        INTAKE,
        SUCK,
        SPIT,
        TILT,
        DROP_OUTTAKE,
        TRANSFER,
        STOW,

        IDLE2,
        EXTEND,
        RELOCALIZE,

        IDLE3,
        SCORE,
        RETRACT,

        INTAKE_AGAIN,
        STOWANGLE,
        ALIGN,
        TO_STACK
    }


    int slideLevel = 1;
    int turretLevel = -1;
    boolean manualSlides = false;

    boolean leftClosed = true;
    boolean rightClosed = true;

    boolean hangReady = false;
    boolean extended = false;
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

    boolean slideBumper = true;
    boolean imuReset = false;
    FtcDashboard dashboard;

    public static String loc = "left";
    public static boolean middlePark = false;
    private VisionPortal portal;
    private RedPropThreshold redFarPropThreshold;
    private AprilTagRelocalize relocalize;
    private int delay = 10000;
    private int exposure = 6;
    private int gain = 100;
    private double placementY=-43;
    private int cycles = 0;
    private int numCycles=2;
    private double placePause = 0.9;
    //private int tagPose = 3;
    private Pose2d relocalizePose;
    private KALMAN kalman;
    private int intakeNum = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Bulk Reading
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        redFarPropThreshold = new RedPropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size( 1280, 720))
                .addProcessor(redFarPropThreshold)
                .build();


        robot = new LebronClass(hardwareMap);


        // V4B Motion Profile
        MotionProfile v4bProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(robot.outtake.v4barStow, 0, 0),
                new MotionState(robot.outtake.v4barTransfer, 0, 0),
                25,
                25,
                25
        );

        // Intake Motion Profile
        MotionProfile intakeProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(robot.intake.tiltStow, 0, 0),
                new MotionState(robot.intake.tiltDown, 0, 0),
                25,
                25,
                25
        );
        Pose2d startPose = new Pose2d(16.5, -64, Math.toRadians(-90));
        robot.drive.setPoseEstimate(startPose);
        kalman = new KALMAN(startPose);

        ElapsedTime profileTimer = new ElapsedTime();

        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startPose) // Truss side / No Prop Seen
                // Drive to spike
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(24, -50, Math.toRadians(-90)), Math.toRadians(60))
                .build();
        TrajectorySequence middle = robot.drive.trajectorySequenceBuilder(startPose) // middle
                // Drive to spike
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(20, -31, Math.toRadians(-30)), Math.toRadians(90))
                .build();
        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(startPose) // left
                // Drive to spike
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(12, -40, Math.toRadians(-30)), Math.toRadians(135))
                .build();

        // MAIN State Machine
        StateMachine machine = new StateMachineBuilder()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                .state(LinearStates.PURPLE)
                .onEnter(() -> {
                    switch (loc) {
                        case "none":
                            robot.drive.followTrajectorySequenceAsync(left);
                            placementY=-30;
                            placePause=2.5;
                            //tagPose=3;
                            break;
                        case "right":
                            robot.drive.followTrajectorySequenceAsync(right);
                            placementY=-43;
                            //tagPose=2;
                            break;
                        case "left":
                            robot.drive.followTrajectorySequenceAsync(middle);
                            placementY=-37;
                            //tagPose=1;
                            break;
                    }
                    robot.intake.tiltStow();
                    robot.outtake.v4barPurple();
                    robot.outtake.v4BarAnglePurple();
                })
                .transitionTimed(2, LinearStates.PURPLEPAUSE)
                .state(LinearStates.PURPLEPAUSE)
                .onEnter(() -> {
                    robot.outtake.openRight();
                })
                .transitionTimed(0.5, LinearStates.RELOCALIZE)
                //.transitionTimed(1, LinearStates.EXTEND)
                .state(LinearStates.PURPLE2STACK)
                .onEnter(() -> {
                    robot.outtake.v4barStow();
                    robot.outtake.turretTransfer();
                    robot.drive.followTrajectorySequenceAsync(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-52, -13, Math.toRadians(180)))
                            .build());
                })
                .transition(() -> !robot.drive.isBusy(), LinearStates.IDLE1)
                .state(LinearStates.IDLE1)                 // Driving to wing to pick up
                .transitionTimed(1)

                .state(LinearStates.INTAKE)
                .onEnter( () -> {
                    profileTimer.reset();

                    robot.intake.setTilt(robot.intake.tiltStack-robot.intake.tiltStackInc*intakeNum); // Drop Intake
                    robot.intake.setIntake(1); // Spin Intake
                    robot.outtake.openBothClaws(); // Claw Open
                    robot.outtake.turretTransfer();
                    robot.drive.followTrajectorySequenceAsync(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-60, -36, Math.toRadians(180)))
                            .build());
                })
                .loop( () -> {
                    MotionState intakeState = intakeProfile.get(profileTimer.seconds());
//                    robot.intake.setIntake(intakeState.getX());
                })
                .transitionTimed(1.5) // if let go and not both pixels
//                .transition( () -> robot.intake.getPixel1() && robot.intake.getPixel2())


                .state(LinearStates.SUCK)
                .onEnter( () -> {
                    robot.intake.setIntake(0.8); // keep Intaking
                    robot.intake.tiltUp(); // Intake tilts up
                    robot.outtake.turretTransfer();
                    robot.outtake.v4barAngleTransfer();
                    robot.drive.followTrajectorySequenceAsync(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .splineToConstantHeading(new Vector2d(-36, -60), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(20, -60), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(42, -36), Math.toRadians(45))
                            .build());
                })
                .transitionTimed(0.25)
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.IDLE1) // Intake Again if we missed


                .state(LinearStates.SPIT)
                .onEnter( () -> {
                    robot.intake.setIntake(-0.5);
                })
                .transitionTimed(0.125)
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.IDLE1) // Intake Again if we missed


                .state(LinearStates.TILT)
                .onEnter( () -> {
                    robot.intake.setIntake(0.25); // suck in
                    robot.intake.tiltUp(); // Intake tilts up
                    robot.outtake.turretTransfer();
                })
                .onExit( () -> {
                    profileTimer.reset();
                })
                .transitionTimed(0.25)
//                .transition( () ->  robot.intake.isTiltUp()) // Tilt is up
                .transition( () ->  gamepad1.right_trigger > 0.5 , LinearStates.IDLE1) // Intake Again if we missed



                .state(LinearStates.DROP_OUTTAKE)
                .onEnter( () -> {
                    profileTimer.reset();

                    robot.intake.setIntake(0); // Stop Intake
                    robot.outtake.openBothClaws();
                    robot.outtake.turretTransfer();

//                    robot.outtake.v4barTransfer();
                    robot.outtake.v4barAngleTransfer();

                })
                .loop( () -> {
                    MotionState state = v4bProfile.get(profileTimer.seconds());
                    robot.outtake.setV4Bar(state.getX());
                })
                .transition( () ->  profileTimer.seconds() > v4bProfile.duration()) // V4b is down
//                .transitionTimed(0.4)

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
                    robot.outtake.setV4BarAngle(robot.outtake.angleTransfer+0.015);
                    robot.outtake.turretTransfer();

                    robot.outtake.closeRightMore();
                    robot.outtake.closeLeftMore();

                    robot.intake.setTilt(robot.intake.tiltUp - 0.1);
                    //robot.intake.tiltStow();
                })
                .transitionTimed(0.3)
                .state(LinearStates.STOWANGLE)
                .onEnter( () -> {
                    robot.intake.tiltStow();
                    robot.outtake.v4barAngleStow();
                    robot.outtake.turretTransfer();
                })
                .transitionTimed(0.25, LinearStates.IDLE2)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                .state(LinearStates.IDLE2)
                .transitionTimed(1.25)
                .state(LinearStates.EXTEND)
                .onEnter( () -> {
                    //robot.outtake.slidesToLevel(slideLevel); // Extend Slide
                    robot.outtake.v4barScore(); // V4b Score Position
                    robot.outtake.turretTo(turretLevel);
                    extended = true;
                })
                .onExit( () -> {
                    extended = false;
                })
                .transitionTimed(0.8)
                .state(LinearStates.RELOCALIZE)
                .onEnter(() -> {
                    robot.outtake.v4barScore(); // V4b Score Position
                    robot.outtake.turretTo(turretLevel);
                    robot.drive.followTrajectorySequenceAsync(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(50.5, placementY, Math.toRadians(180)))
                            .build());
                })
                .onExit( () -> {
                    robot.outtake.openBothClaws();
                    placementY=-32;
                })
                .transitionTimed(0.9)
                .state(LinearStates.PAUSE)
                .transitionTimed(0.6)

                .state(LinearStates.RETRACT)
                .onEnter( () -> {
                    placePause=0.8;
                    robot.outtake.v4barStow(); // V4b Stow Position
                    robot.outtake.turretTransfer(); // Turret Vertical
                    robot.outtake.retractSlides(); // Retract Slide
                    robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(),robot.drive.getPoseEstimate().getY()+6,robot.drive.getPoseEstimate().getHeading()));
                    slideLevel = 1;
                    turretLevel = -1;
                    manualSlides = false;
                })
                .onExit( () -> {
                    robot.outtake.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.outtake.midSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.outtake.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                })
                .transitionTimed(0.5)
                .state(LinearStates.TO_STACK)
                .onEnter( () -> {
                    numCycles++;
                    intakeNum+=2;
                    loc = "left";
                    robot.drive.followTrajectorySequenceAsync(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .splineToConstantHeading(new Vector2d(-36, -60), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(20, -60), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(-52, -36), Math.toRadians(135))
                            .build());
                })
                .transitionTimed(3.35, LinearStates.INTAKE)
                // Fail safe
                .state(LinearStates.INTAKE_AGAIN)
                .onEnter( () -> {
                    robot.outtake.setV4Bar(0.5); // V4b Stow Position
                    robot.outtake.v4barAngle.setPosition(0.7); // V4b Stow Position

                })
                .transitionTimed(0.5, LinearStates.IDLE1)
                .build();


        robot.outtake.closeBothClaws();
        robot.outtake.turretTransfer();
        robot.intake.tiltUp();

        while (opModeInInit() && !isStopRequested()) {
            loc = redFarPropThreshold.getPropPosition();
            telemetry.addData("Prop Position", redFarPropThreshold.getPropPosition());
            telemetry.addData("Avg Left Box Value", redFarPropThreshold.getAvergageLeft());
            telemetry.addData("Avg Right Box Value", redFarPropThreshold.getAvergageRight());

            if (gamepad1.a) {
                middlePark = true;
            } else if (gamepad1.b) {
                middlePark = false;
            }
            telemetry.addData("Ideal Park?", middlePark);

            telemetry.update();
        }
        portal.close();
        relocalize = new AprilTagRelocalize(hardwareMap);

        waitForStart();

//        robot.special.holdDrone();
        machine.start();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            if (extended) {
                //relocalize.setManualExposure(exposure, gain);
                relocalizePose = relocalize.getTagPos(new int[]{1,2,3});
                Log.d("Relocalize Pose: ", relocalizePose.toString());
//                if (relocalizePose.getX()<=72 && read) {
//                    relocalizePose = new Pose2d(relocalizePose.getX(), relocalizePose.getY(), robot.drive.getPoseEstimate().getHeading());
//                    robot.drive.setPoseEstimate(relocalizePose);
//                }
                if (relocalizePose.getX()<=72) {
                    relocalizePose = new Pose2d(relocalizePose.getX(), relocalizePose.getY(), robot.drive.getPoseEstimate().getHeading());
                    kalman.update(robot.drive.getPoseEstimate(), relocalizePose);
                    Log.d("Kalman Pose: ", kalman.getPose().toString());
                    Log.d("Odometry Pose: ", robot.drive.getPoseEstimate().toString());
                    Pose2d input = kalman.getPose();
                    input = new Pose2d(input.getX(), input.getY(), robot.drive.getPoseEstimate().getHeading());
                    robot.drive.setPoseEstimate(input);
                }
            }
            if (numCycles>cycles) {
                break;
            }
            machine.update();
            robot.drive.update();
            // Will run one bulk read per cycle,
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            if (robot.outtake.leftSlide.getVelocity() < 2 && robot.outtake.leftSlide.getCurrentPosition() < 0) {
                robot.outtake.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (robot.outtake.midSlide.getVelocity() < 2 && robot.outtake.midSlide.getCurrentPosition() < 0) {
                robot.outtake.midSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (robot.outtake.rightSlide.getVelocity() < 2 && robot.outtake.rightSlide.getCurrentPosition() < 0) {
                robot.outtake.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // Telemetry
            telemetry.addData("State", machine.getState());
            telemetry.addData("Loc", loc);
            telemetry.addData("Pose", robot.drive.getPoseEstimate());
            telemetry.addData("Slide Level", slideLevel);
            telemetry.addData("slide bumper adjust", slideBumper);
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

            telemetry.addData("Heading Degrees", Math.toDegrees(currAngle));
            machine.update();
            // in da loop
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;

            telemetry.update();

        }
        relocalize.visionPortal.close();
        robot.drive.followTrajectorySequence(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(51, -62, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(64, -64, Math.toRadians(180)))
                .build());
    }
}

