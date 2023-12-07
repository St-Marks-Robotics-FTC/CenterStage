package org.firstinspires.ftc.teamcode.armbot;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp
public class servoTest extends LinearOpMode {

//    public static int motorPos = 0;
//    public static double motorSpeed = 0.7;

    public static String servoName = "claw1";

    public static Double pos = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        Servo servo = hardwareMap.get(Servo.class, servoName);




        if (isStopRequested()) return;

        while (opModeIsActive()) {


            servo.setPosition(pos);




            telemetry.addData("Name: ", servoName);
            telemetry.addData("Position: ", pos);

            telemetry.update();
        }
    }
}
