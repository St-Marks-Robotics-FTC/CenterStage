package org.firstinspires.ftc.teamcode.Testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
@TeleOp
public class TwoDistanceTest extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        Rev2mDistanceSensor leftDistance = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeft2m");
        Rev2mDistanceSensor rightDistance = hardwareMap.get(Rev2mDistanceSensor.class, "distanceRight2m");
        if (isStopRequested()) return;

        while (opModeIsActive()) {


            double leftDist = leftDistance.getDistance(DistanceUnit.CM);
            double rightDist = rightDistance.getDistance(DistanceUnit.CM);




            telemetry.addData("Left Distance: ", leftDist);
            telemetry.addData("Right Distance: ", rightDist);

            telemetry.update();
        }
    }
}
