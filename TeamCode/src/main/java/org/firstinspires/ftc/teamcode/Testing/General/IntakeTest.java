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

import org.firstinspires.ftc.teamcode.LM2.Roadrunner.MecanumDrive;


@Config
@TeleOp(group = "General")
public class IntakeTest extends LinearOpMode {

//    public static int motorPos = 0;
//    public static double motorSpeed = 0.7;

    public static String motorName = "intake";
    public static String servo1Name = "linkageLeft";
    public static String servo2Name = "linkageRight";

    public static Double pos = 0.5;
    public static Double up = 0.4;
    public static Double down = 0.6;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        DcMotor motor = hardwareMap.dcMotor.get(motorName);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ServoImplEx servo1 = hardwareMap.get(ServoImplEx.class, servo1Name);
        ServoImplEx servo2 = hardwareMap.get(ServoImplEx.class, servo2Name);

        servo2.setDirection(Servo.Direction.REVERSE);





        if (isStopRequested()) return;

        while (opModeIsActive()) {

            motor.setPower(-gamepad1.right_stick_y);

            if (gamepad1.a) {
                pos = up;
            }

            if (gamepad1.b) {
                pos = down;
            }
            pos+=gamepad1.right_stick_y/1000;

            servo1.setPosition(pos);
            servo2.setPosition(pos);


            telemetry.addData("Name 1", servo1Name);
            telemetry.addData("Name 2", servo2Name);
            telemetry.addData("Position", pos);



            telemetry.addData("Name: ", motorName);
            telemetry.addData("Motor Position: ", motor.getCurrentPosition());
            telemetry.addData("Power: ", motor.getPower());

            telemetry.update();
        }
    }
}
