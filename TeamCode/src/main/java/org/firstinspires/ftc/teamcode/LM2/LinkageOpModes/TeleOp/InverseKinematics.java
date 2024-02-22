package org.firstinspires.ftc.teamcode.LM2.LinkageOpModes.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp(group = "General")
public class InverseKinematics extends LinearOpMode {

//    public static int motorPos = 0;
//    public static double motorSpeed = 0.7;

    public static String motorName = "arm";
    double COUNTS_PER_REVOLUTION = 537.7 * 5;
    public static int zero90 = 120;

    public static Double servoPos = 0.5;
    public static Double servoAngleOffset = 35.0;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        DcMotor motor = hardwareMap.dcMotor.get(motorName);
        Servo servo = hardwareMap.servo.get("wrist");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





        if (isStopRequested()) return;

        while (opModeIsActive()) {


            double motorAngle = (motor.getCurrentPosition() - zero90) / COUNTS_PER_REVOLUTION * 360; // angle from horizontal

            motorAngle += 90; // angle from vertical

            telemetry.addData("Motor Ticks: ", motor.getCurrentPosition());
            telemetry.addData("Motor Angle: ", motorAngle);


            servo.setPosition(servoPos);

            double position = servo.getPosition();
            double servoAngle = map(position, 0, 1, 0, 180);

            servoAngle += servoAngleOffset;

            telemetry.addData("Position", position);
            telemetry.addData("Servo Angle", servoAngle);


            // Parallel to ground
//            double desiredAngle = motorAngle + 90 - servoAngleOffset;
//            double desiredPosition = map(desiredAngle, 0, 180, 0, 1);
//            servo.setPosition(desiredPosition);

            // Parallel to board
//            double desiredAngle = motorAngle + 30 - servoAngleOffset;
//            double desiredPosition = map(desiredAngle, 0, 180, 0, 1);
//            servo.setPosition(desiredPosition);




            telemetry.update();
        }

    }
    // Map a value from one range to another
    private double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
