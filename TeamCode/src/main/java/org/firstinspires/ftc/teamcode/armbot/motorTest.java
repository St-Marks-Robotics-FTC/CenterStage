package org.firstinspires.ftc.teamcode.armbot;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp
public class motorTest extends LinearOpMode {

//    public static int motorPos = 0;
//    public static double motorSpeed = 0.7;

    public static String motorName = "arm";


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





        if (isStopRequested()) return;

        while (opModeIsActive()) {


            motor.setPower(-gamepad1.left_stick_y);





            telemetry.addData("Name: ", motorName);
            telemetry.addData("Motor Position: ", motor.getCurrentPosition());
            telemetry.addData("Power: ", motor.getPower());

            telemetry.update();
        }
    }
}
