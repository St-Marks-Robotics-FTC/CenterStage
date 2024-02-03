package org.firstinspires.ftc.teamcode.Testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Disabled

@Config
@TeleOp
public class LiftTest extends LinearOpMode {

//    public static int motorPos = 0;
//    public static double motorSpeed = 0.7;

    public static String motorName = "slideLeft";
    public static String motorName2 = "slideRight";


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        DcMotor motor = hardwareMap.dcMotor.get(motorName);
        DcMotor motor2 = hardwareMap.dcMotor.get(motorName2);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);






        if (isStopRequested()) return;

        while (opModeIsActive()) {


            motor.setPower(-gamepad1.left_stick_y);
            motor2.setPower(-gamepad1.left_stick_y);






            telemetry.addData("Name: ", motorName);
            telemetry.addData("Motor Position: ", motor.getCurrentPosition());
            telemetry.addData("Power: ", motor.getPower());

            telemetry.update();
        }
    }
}
