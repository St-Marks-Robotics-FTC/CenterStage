package org.firstinspires.ftc.teamcode.Testing.General;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp(group = "General")
public class ServoTest extends LinearOpMode {

//    public static int motorPos = 0;
//    public static double motorSpeed = 0.7;

    public static String servoName = "turret";

    public static Double pos = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        Servo servo = hardwareMap.get(Servo.class, servoName);

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.x) {
                pos = 0.0;
            }

            if (gamepad1.b) {
                pos = 0.5;
            }


            servo.setPosition(pos);




            telemetry.addData("Name: ", servoName);
            telemetry.addData("Position: ", pos);

            telemetry.update();
        }
    }
}
