package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Config
@TeleOp
public class ActuatorTest extends OpMode {

    boolean pid = true;
    DcMotorEx slideLeft;
    DcMotorEx slideRight;
    public static String leftName = "leftActuator";
    public static String rightName= "rightActuator";

    private double power = 1.0;
    public static int position = 8000;

    public double correctionLeft = 0.99;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slideLeft = hardwareMap.get(DcMotorEx.class, leftName);
        slideRight = hardwareMap.get(DcMotorEx.class, rightName);
        //slideLeft.setDirection(DcMotorEx.Direction.REVERSE);
        //slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            pid = true;
            slideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            slideLeft.setTargetPosition(0);
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideLeft.setPower(correctionLeft*power);
            slideRight.setTargetPosition(0);
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setPower( power);
        } else if (gamepad1.y) {
            pid = true;
            slideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            slideLeft.setTargetPosition(position);
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideLeft.setPower(correctionLeft*power);
            slideRight.setTargetPosition(position);
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setPower( power);
        }
        if (gamepad1.left_trigger > 0.1) {
            pid = false;
            slideLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            slideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            slideLeft.setPower(-gamepad1.left_trigger);
            slideRight.setPower(-gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > 0.1) {
            pid = false;
            slideLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            slideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            slideLeft.setPower(gamepad1.right_trigger);
            slideRight.setPower(gamepad1.right_trigger);
        } else if (!pid){
            slideLeft.setPower(0);
            slideRight.setPower(0);
        }
//        if (gamepad1.b) {
//            slideLeft.setPower(-1.0);
//            slideRight.setPower(-1.0);
//        }

//        else {
//            slideLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            slideRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        }
        if (gamepad1.dpad_down) { // reset encoders
            slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        telemetry.addData("Slide Left: ", slideLeft.getCurrentPosition());
        telemetry.addData("Slide Right: ", slideRight.getCurrentPosition());
        telemetry.addData("Slide Left Current: ", slideLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Slide Right Current: ", slideRight.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}
