package org.firstinspires.ftc.teamcode.Fallback.Opmodes.Testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@Config
@TeleOp
public class ServoSync extends LinearOpMode {


    public static String servo1Name = "clawLeft";
    public static String servo2Name = "clawRight";

    public static Double pos = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        ServoImplEx servo1 = hardwareMap.get(ServoImplEx.class, servo1Name);
        ServoImplEx servo2 = hardwareMap.get(ServoImplEx.class, servo2Name);

        servo1.setDirection(Servo.Direction.REVERSE);




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