package org.firstinspires.ftc.teamcode.Testing.General;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


//@Disabled
@Config
@TeleOp(group = "General")

public class ServoSync extends LinearOpMode {


    public static String servo1Name = "linkageLeft";
    public static String servo2Name = "linkageRight";

    public static Double pos = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        ServoImplEx servo1 = hardwareMap.get(ServoImplEx.class, servo1Name);
        ServoImplEx servo2 = hardwareMap.get(ServoImplEx.class, servo2Name);

        servo2.setDirection(Servo.Direction.REVERSE);




        if (isStopRequested()) return;

        while (opModeIsActive()) {


            servo1.setPosition(pos);
            servo2.setPosition(pos);


            telemetry.addData("Name 1", servo1Name);
            telemetry.addData("Name 2", servo2Name);
            telemetry.addData("Position", pos);

            telemetry.update();
        }
    }
}
