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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;
import org.firstinspires.ftc.teamcode.LebronBot.Subsystems.KALMAN;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.LebronBot.Subsystems.DistanceRelocalize;
import org.firstinspires.ftc.teamcode.Vision.AprilTag.AprilTagRelocalize;
import org.firstinspires.ftc.teamcode.Vision.Prop.BlueFarPropThreshold;
import org.firstinspires.ftc.teamcode.Vision.Prop.RedFarPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Config
@Autonomous
public class RedFarSTATE extends  LinearOpMode{
    enum LinearStates {
        PURPLE,
        PURPLEPAUSE,
        PURPLE2STACK,
        DISTANCERELOCALIZE,
        PAUSE,
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
        TO_STACK,
        SUCKY
    }


    int slideLevel = 1;
    int turretLevel = 0;
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
    private RedFarPropThreshold redFarPropThreshold;
    private AprilTagRelocalize relocalize;
    private int delay = 10000;
    private int exposure = 6;
    private int gain = 100;
    private double placementY=43;
    private int cycles = 2;
    private int numCycles=0;
    private int tagPose = 3;
    private boolean read = false;
    private Pose2d relocalizePose;
    private KALMAN kalman;
    private int intakeNum = 4;
    private DistanceRelocalize ak47;
    private double intakeDistance=-56.5;
    private double purplePause=1.7;
    private double intakeTime = 0.7;
    private int slideHeight = 100;
    private boolean purpleIntake = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Bulk Reading
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        redFarPropThreshold = new RedFarPropThreshold();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(redFarPropThreshold)
                .build();


        robot = new LebronClass(hardwareMap);
        ak47 = new DistanceRelocalize(hardwareMap, DistanceRelocalize.Side.RED);

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
        Pose2d startPose = new Pose2d(-40.5, -64, Math.toRadians(-90));
        robot.drive.setPoseEstimate(startPose);
        kalman = new KALMAN(startPose);

        ElapsedTime profileTimer = new ElapsedTime();

        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(startPose) // Truss side / No Prop Seen
                // Drive to spike
                .setReversed(true)
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(-38, -43, Math.toRadians(-150)), Math.toRadians(60))
                .build();
        TrajectorySequence middle = robot.drive.trajectorySequenceBuilder(startPose) // middle
                // Drive to spike
                .setReversed(true)
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(-46, -30, Math.toRadians(-160)), Math.toRadians(120))
                .build();
        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(startPose) // left
                // Drive to spike
                .setReversed(true)
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(-56, -21, Math.toRadians(135)), Math.toRadians(120))
                .build();

        // MAIN State Machine
        StateMachine machine = new StateMachineBuilder()
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                .state(LinearStates.PURPLE)
                .onEnter(() -> {
                    switch (loc) {
                        case "none":
                            robot.drive.followTrajectorySequenceAsync(right);
                            placementY = -43;
                            turretLevel = 2;
                            purplePause = 1.7;
                            break;
                        case "right":
                            robot.drive.followTrajectorySequenceAsync(middle);
                            placementY = -37;
                            turretLevel = 3;
                            purplePause = 2.3;
                            break;
                        case "left":
                            robot.drive.followTrajectorySequenceAsync(left);
                            placementY = -32;
                            turretLevel=-3;
                            break;
                    }
                    robot.outtake.v4barPurple();
                    robot.outtake.v4BarAnglePurple();
                    robot.intake.tiltStow();
                })
                .transitionTimed(1.8)
                .state(LinearStates.PURPLEPAUSE)
                .onEnter(() -> {
                    robot.outtake.openRight();
                })
                .transitionTimed(0.3, LinearStates.PURPLE2STACK)
                //.transitionTimed(1, LinearStates.EXTEND)
                .state(LinearStates.PURPLE2STACK)
                .onEnter(() -> {
                    robot.outtake.v4barStow();
                    robot.outtake.turretTransfer();
                    robot.drive.followTrajectorySequenceAsync(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-52, -12, Math.toRadians(180)))
                            .build());
                })
                .transitionTimed(1.5, LinearStates.INTAKE)
                .state(RedFarSTATE.LinearStates.DISTANCERELOCALIZE)
                .onEnter(() -> read = true)
                .onExit(() -> read = false)
                .transitionTimed(0.4)

                //.state(LinearStates.DISTANCERELOCALIZE)
                //.onEnter(() -> robot.drive.setPoseEstimate(ak47.relocalize()))
                .state(LinearStates.INTAKE)
                .onEnter(() -> {
                    profileTimer.reset();

                    robot.intake.setStack(intakeNum); // Drop Intake
                    robot.intake.setIntake(1); // Spin Intake
                    robot.outtake.openBothClaws(); // Claw Open
                    robot.outtake.turretTransfer();
                    robot.drive.followTrajectorySequenceAsync(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(intakeDistance, -14, Math.toRadians(180)))
                            .build());
                })
                //.transitionTimed(1.5) // if let go and not both pixels
                .transitionTimed(0.7)
                .state(LinearStates.SUCKY)
                .transitionTimed(0.4)

                .state(LinearStates.SUCK)
                .onEnter(() -> {
                    robot.intake.setIntake(0.8); // keep Intaking
                    robot.intake.tiltUp(); // Intake tilts up
                    robot.outtake.turretTransfer();
                    robot.outtake.v4barAngleTransfer();
                    robot.drive.followTrajectorySequenceAsync(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .setTangent(0)
//                            .splineToConstantHeading(new Vector2d(5, 10), Math.toRadians(0))
//                            .splineToConstantHeading(new Vector2d(44, 36), Math.toRadians(35))
                            .splineToSplineHeading(new Pose2d(4, -12, Math.toRadians(180)), Math.toRadians(0))
                            .splineToSplineHeading(new Pose2d(44, -34.5, Math.toRadians(180)), Math.toRadians(-35))
                            .build());
                    intakeNum=1;
                })
                .transitionTimed(0.25)
                .transition(() -> gamepad1.right_trigger > 0.5, LinearStates.IDLE1) // Intake Again if we missed


                .state(LinearStates.SPIT)
                .onEnter(() -> {
                    robot.intake.setIntake(-0.5);
                })
                .transitionTimed(0.2)
                .transition(() -> gamepad1.right_trigger > 0.5, LinearStates.IDLE1) // Intake Again if we missed


                .state(LinearStates.TILT)
                .onEnter(() -> {
                    robot.intake.setIntake(0.25); // suck in
                    robot.intake.tiltUp(); // Intake tilts up
                    robot.outtake.turretTransfer();
                })
                .onExit(() -> {
                    profileTimer.reset();
                })
                .transitionTimed(0.25)
//                .transition( () ->  robot.intake.isTiltUp()) // Tilt is up
                .transition(() -> gamepad1.right_trigger > 0.5, LinearStates.IDLE1) // Intake Again if we missed


                .state(LinearStates.DROP_OUTTAKE)
                .onEnter(() -> {
                    profileTimer.reset();

                    robot.intake.setIntake(0); // Stop Intake
                    robot.outtake.openBothClaws();
                    robot.outtake.turretTransfer();

//                    robot.outtake.v4barTransfer();
                    robot.outtake.v4barAngleTransfer();

                })
                .loop(() -> {
                    MotionState state = v4bProfile.get(profileTimer.seconds());
                    robot.outtake.setV4Bar(state.getX());
                })
                .transition(() -> profileTimer.seconds() > v4bProfile.duration()) // V4b is down
//                .transitionTimed(0.4)

                .state(LinearStates.TRANSFER)
                .onEnter(() -> {
                    robot.intake.setIntake(0); // Stop Intake
                    robot.outtake.closeBothClaws(); // Claw Grab
                    robot.outtake.turretTransfer();
                })
                .transitionTimed(0.3)
                .transition(() -> gamepad1.right_trigger > 0.5, LinearStates.IDLE1) // Intake Again if we missed

                .state(LinearStates.STOW)
                .onEnter(() -> {
                    robot.outtake.setV4Bar(robot.outtake.v4barStow); // V4b Stow Position
                    robot.outtake.setV4BarAngle(robot.outtake.angleTransfer + 0.015);
                    robot.outtake.turretTransfer();

                    robot.outtake.closeRightMore();
                    robot.outtake.closeLeftMore();

                    robot.intake.setTilt(robot.intake.tiltUp - 0.1);
                    //robot.intake.tiltStow();
                })
                .transitionTimed(0.3)
                .state(LinearStates.STOWANGLE)
                .onEnter(() -> {
                    robot.intake.tiltStow();
                    robot.outtake.v4barAngleStow();
                    robot.outtake.turretTransfer();
                })
                .transitionTimed(0.25, LinearStates.IDLE2)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                .state(LinearStates.IDLE2)
                .onEnter(()-> {
                    robot.outtake.v4barScore(); // V4b Score Position
                    robot.outtake.turretTo(turretLevel);
                    robot.outtake.setSlides(slideHeight);
                })
                .transitionTimed(0.8)
                //.transition(() -> !robot.drive.isBusy())
                .state(LinearStates.EXTEND)
                .onEnter(() -> {
                    //robot.outtake.slidesToLevel(slideLevel); // Extend Slide
                    extended = false;
                })
                .onExit(() -> {
                    extended = false;
                })
                .transitionTimed(0.4)
                .state(LinearStates.RELOCALIZE)
                .onEnter(() -> {
                    robot.drive.followTrajectorySequenceAsync(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(50.5, placementY, Math.toRadians(180)))
                            .build());
                })
                .onExit(() -> {
                    robot.outtake.openLeft();
                })
                .transitionTimed(0.6)
                .state(LinearStates.PAUSE)
                .onExit(() -> robot.outtake.openBothClaws())
                .transitionTimed(0.3)
                .state(LinearStates.IDLE1)
                .transitionTimed(0.3)
                .state(LinearStates.RETRACT)
                .onEnter(() -> {
                    numCycles++;
                    robot.drive.followTrajectorySequenceAsync(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(48, placementY, Math.toRadians(180)))
                            .build());
                    slideLevel = 1;
                    turretLevel = 2;
                    manualSlides = false;
                    //robot.drive.setPoseEstimate(new Pose2d(robot.drive.getPoseEstimate().getX(),robot.drive.getPoseEstimate().getY()+3,robot.drive.getPoseEstimate().getHeading()));
                    placementY = -32;
                    slideHeight=250;
                })
                .onExit(() -> {
                    robot.outtake.v4barStow(); // V4b Stow Position
                    robot.outtake.turretTransfer(); // Turret Vertical
                    robot.outtake.retractSlides(); // Retract Slide
                    robot.outtake.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.outtake.midSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.outtake.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                })
                .transitionTimed(0.35)
                .state(LinearStates.TO_STACK)
                .onEnter(() -> {
                    loc = "left";
                    robot.drive.followTrajectorySequenceAsync(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .setTangent(Math.toRadians(140))
//                            .splineToConstantHeading(new Vector2d(4, 8), Math.toRadians(180))
//                            .splineToConstantHeading(new Vector2d(-50,  10), Math.toRadians(180))
                            .splineToSplineHeading(new Pose2d(8, -9, Math.toRadians(180)), Math.toRadians(180))
                            .splineToSplineHeading(new Pose2d(-46, -13, Math.toRadians(180)), Math.toRadians(180))
                            .build());
                    intakeDistance = -57.5;
                })
                .transitionTimed(3.25, LinearStates.DISTANCERELOCALIZE)
                // Fail safe
                .state(LinearStates.INTAKE_AGAIN)
                .onEnter(() -> {
                    robot.outtake.setV4Bar(0.5); // V4b Stow Position
                    robot.outtake.v4barAngle.setPosition(0.7); // V4b Stow Position

                })
                .transitionTimed(0.5, LinearStates.IDLE1)
                .build();


        robot.outtake.closeRightMore();
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
                relocalizePose = relocalize.getTagPos(new int[]{1, 2, 3});
                Log.d("Relocalize Pose: ", relocalizePose.toString());
//                if (relocalizePose.getX()<=72 && read) {
//                    relocalizePose = new Pose2d(relocalizePose.getX(), relocalizePose.getY(), robot.drive.getPoseEstimate().getHeading());
//                    robot.drive.setPoseEstimate(relocalizePose);
//                }
                if (relocalizePose.getX() <= 72) {
                    relocalizePose = new Pose2d(relocalizePose.getX(), relocalizePose.getY(), robot.drive.getPoseEstimate().getHeading());
                    kalman.update(robot.drive.getPoseEstimate(), relocalizePose);
                    Log.d("Kalman Pose: ", kalman.getPose().toString());
                    Log.d("Odometry Pose: ", robot.drive.getPoseEstimate().toString());
                    Pose2d input = kalman.getPose();
                    input = new Pose2d(input.getX(), input.getY(), robot.drive.getPoseEstimate().getHeading());
                    robot.drive.setPoseEstimate(input);
                }
            }
            if (read) {
                Pose2d sensorytouch = ak47.relocalize(robot.drive.getPoseEstimate().getHeading());
                Log.d("Sensory: ", sensorytouch.toString());
                if (sensorytouch.vec().minus(robot.drive.getPoseEstimate().vec()).norm() < 10) {
                    kalman.update(robot.drive.getPoseEstimate(), sensorytouch);
                    Log.d("Kalman Pose: ", kalman.getPose().toString());
                    Log.d("Odometry Pose: ", robot.drive.getPoseEstimate().toString());
                    Pose2d input = kalman.getPose();
                    input = new Pose2d(robot.drive.getPoseEstimate().getX(), input.getY(), robot.drive.getPoseEstimate().getHeading());
                    robot.drive.setPoseEstimate(input);
                }
            }
            if (numCycles > cycles) {
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
            Log.d("ROBOTPOSE: ", robot.drive.getPoseEstimate().toString());
            //telemetry.addData("intakeTime: ", intakeTime);
            telemetry.addData("State", machine.getState());
            telemetry.addData("Loc", loc);
            telemetry.addData("Tag Pose ", tagPose);
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
        robot.outtake.v4barStow(); // V4b Stow Position
        robot.outtake.turretTransfer(); // Turret Vertical
        robot.drive.followTrajectorySequence(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(180)))
                .build());
    }
}
