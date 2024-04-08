package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;


@Config
@TeleOp(group = "General")
public class PlacementTest extends LinearOpMode {

    LebronClass robot;
//    public static int motorPos = 0;
//    public static double motorSpeed = 0.7;


    boolean leftClosed = true;
    boolean rightClosed = true;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new LebronClass(hardwareMap);

        GamepadEx pad1 = new GamepadEx(gamepad1);

        TriggerReader leftTrigger = new TriggerReader(
                pad1, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        TriggerReader rightTrigger = new TriggerReader(
                pad1, GamepadKeys.Trigger.RIGHT_TRIGGER
        );

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Claws
            if (leftTrigger.wasJustPressed()) {
                leftClosed = !leftClosed;
            }
            if (rightTrigger.wasJustPressed()) {
                rightClosed = !rightClosed;
            }

            if (leftClosed) {
                robot.outtake.closeLeftMore();
                leftClosed = true;
            } else {
                robot.outtake.openLeft();
                leftClosed = false;
            }
            if (rightClosed) {
                robot.outtake.closeRightMore();
                rightClosed = true;
            } else {
                robot.outtake.openRight();
                rightClosed = false;
            }




            pad1.readButtons();
            leftTrigger.readValue();
            rightTrigger.readValue();


            telemetry.update();
        }
    }
}
