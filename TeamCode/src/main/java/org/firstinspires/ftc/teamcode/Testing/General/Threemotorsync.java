package org.firstinspires.ftc.teamcode.Testing.General;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Config
@TeleOp(group = "General")
public class Threemotorsync extends LinearOpMode {

//    public static int motorPos = 0;
//    public static double motorSpeed = 0.7;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "liftLeft");
        DcMotorEx mid = hardwareMap.get(DcMotorEx.class, "liftMid");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "liftRight");

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

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            left.setPower(-gamepad1.left_stick_y);
            mid.setPower(-gamepad1.left_stick_y);
            right.setPower(-gamepad1.left_stick_y);

            telemetry.addData("Left Motor Position: ", left.getCurrentPosition());
            telemetry.addData("Left Power: ", left.getPower());
            telemetry.addData("Left Current: ", left.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Mid Motor Position: ", mid.getCurrentPosition());
            telemetry.addData("Mid Power: ", mid.getPower());
            telemetry.addData("Mid Current: ", mid.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Right Motor Position: ", right.getCurrentPosition());
            telemetry.addData("Right Power: ", right.getPower());
            telemetry.addData("Right Current: ", right.getCurrent(CurrentUnit.MILLIAMPS));


            telemetry.update();
        }
    }
}
