package org.firstinspires.ftc.teamcode.Testing.General;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled
@Config
@TeleOp
public class CRServoTest extends LinearOpMode {

//    public static int motorPos = 0;
//    public static double motorSpeed = 0.7;

    public static String servoName = "test";

    public static Double power = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        CRServo servo = hardwareMap.get(CRServo.class, servoName);




        if (isStopRequested()) return;

        while (opModeIsActive()) {


            servo.setPower(power);




            telemetry.addData("Name: ", servoName);
            telemetry.addData("Position: ", power);

            telemetry.update();
        }
    }
}
