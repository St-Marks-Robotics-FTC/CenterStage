package org.firstinspires.ftc.teamcode.RoverGhopla;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "RoverTele")
public class Drivetrain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BozoBot bozoBot = new BozoBot(hardwareMap);

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + rx + x;
            double backLeftPower = y + rx - x;
            double frontRightPower = y - rx - x;
            double backRightPower = y - rx + x;

            if (gamepad1.left_bumper){
                frontLeftPower = 0.3*frontLeftPower;
                backRightPower = 0.3*backRightPower;
                frontRightPower = 0.3*frontRightPower;
                backLeftPower = 0.3*backLeftPower;
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad1.right_bumper){
                bozoBot.ArmUp();
            }else if (gamepad1.right_trigger > 0.5){
                bozoBot.ArmDown();
            }
            //gamepad1.right_bumper = !gamepad1.right_bumper;

            if (gamepad1.y){
                bozoBot.LeftOpen();
                bozoBot.RightOpen();
            }else if(gamepad1.a){
                bozoBot.LeftClose();
                bozoBot.RightClose();
            }
            //gamepad1.y = !gamepad1.y;

            //if (gamepad1.x){
                //bozoBot.LeftOpen();
            //}else {
                //bozoBot.LeftClose();
            //}
            //gamepad1.x = !gamepad1.x;

            //if (gamepad1.b){
                //bozoBot.RightOpen();
            //}else{
                //bozoBot.RightClose();
            //}
            //gamepad1.b = !gamepad1.b;

            telemetry.addData("ArmPosition", bozoBot.arm.getCurrentPosition());
            telemetry.addData("LeftClawPosition", bozoBot.leftclaw.getPosition());
            telemetry.addData("RightClawPosition", bozoBot.rightclaw.getPosition());
            telemetry.update();

        }
    }
}