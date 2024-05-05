package org.firstinspires.ftc.teamcode.Testing.General;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.MecanumDrive;

import java.util.concurrent.TimeUnit;


@Config
@TeleOp(group = "General")
public class UnitTest extends LinearOpMode {

//    public static int motorPos = 0;
//    public static double motorSpeed = 0.7;

    public static String motorName = "intake";
    public static String servo1Name = "linkageLeft";
    public static String servo2Name = "linkageRight";


    public static Double pos = 0.36;
    public static Double up = 0.44;
    public static Double down = 0.27;
    public static Integer increment = 10;
    public static Integer targetPos = 0;
    public static Double slidePower = 0.5;

    public static double modifier = 1;

    private static boolean justPressedIntake = false;
    private static boolean justPressedOuttake = false;
    private static boolean delay1=false;
    private static boolean delay2=false;
    public static Integer delay = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        DcMotor left = hardwareMap.dcMotor.get("leftLift");
        DcMotor mid = hardwareMap.dcMotor.get("midLift");
        DcMotor right = hardwareMap.dcMotor.get("rightLift");

        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mid.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mid.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mid.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor motor = hardwareMap.dcMotor.get(motorName);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ServoImplEx servo1 = hardwareMap.get(ServoImplEx.class, servo1Name);
        ServoImplEx servo2 = hardwareMap.get(ServoImplEx.class, servo2Name);

        servo2.setDirection(Servo.Direction.REVERSE);
        MecanumDrive drive = new MecanumDrive(hardwareMap);




        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*modifier,
                            -gamepad1.left_stick_x*modifier,
                            -gamepad1.right_stick_x*modifier
                    )
            );

            drive.update();

            if (gamepad1.y) {
                motor.setPower(-1);
                pos=down;
                justPressedIntake = true;
            }
            if (!gamepad1.y && justPressedIntake) {
                pos=up;
                motor.setPower(1);
                justPressedIntake = false;
                time.reset();
                delay1 = true;
            }
            if (time.time(TimeUnit.MILLISECONDS)>delay && delay1) {
                motor.setPower(-1);
                delay1=false;
                delay2=true;
            }
            if (time.time(TimeUnit.MILLISECONDS)>delay*2 && delay2) {
                delay2=false;
                motor.setPower(0);
            }

            if (gamepad1.x) {
                motor.setPower(1);
                justPressedOuttake=true;
            }

            if (justPressedOuttake && !gamepad1.x) {
                motor.setPower(0);
                justPressedOuttake=false;
            }

            if (gamepad1.a) {
                pos = up;
            }

            if (gamepad1.b) {
                pos = down;
            }

            if (gamepad1.left_trigger>0) {
                pos-=0.01;
            }

            if (gamepad1.left_bumper) {
                pos+=0.01;
            }

            if (gamepad1.right_bumper) {
                modifier = 0.3;
            } else {
                modifier = 1;
            }

            servo1.setPosition(pos);
            servo2.setPosition(pos);

            if (false) {
                targetPos=Math.min(targetPos+increment, 1000);
            }
            if (false) {
                targetPos=Math.max(targetPos-increment, 0);
            }

            left.setTargetPosition(targetPos);
            mid.setTargetPosition(targetPos);
            right.setTargetPosition(targetPos);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setPower(slidePower);
            mid.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mid.setPower(slidePower);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setPower(slidePower);

            telemetry.addData("Name 1", servo1Name);
            telemetry.addData("Name 2", servo2Name);
            telemetry.addData("Position", pos);
            telemetry.addData("Time: ", time.milliseconds());

            telemetry.addData("Left Motor Position: ", left.getCurrentPosition());
            telemetry.addData("Left Power: ", left.getPower());
            telemetry.addData("Mid Motor Position: ", mid.getCurrentPosition());
            telemetry.addData("Mid Power: ", mid.getPower());
            telemetry.addData("Right Motor Position: ", right.getCurrentPosition());
            telemetry.addData("Right Power: ", right.getPower());

            telemetry.addData("Name: ", motorName);
            telemetry.addData("Motor Position: ", motor.getCurrentPosition());
            telemetry.addData("Power: ", motor.getPower());

            telemetry.update();
        }
    }
}
