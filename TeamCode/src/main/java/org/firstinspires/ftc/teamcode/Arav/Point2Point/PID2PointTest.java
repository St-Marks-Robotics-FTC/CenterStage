package org.firstinspires.ftc.teamcode.Arav.Point2Point;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.MecanumDrive;

@Config
@TeleOp
public class PID2PointTest extends LinearOpMode {

    public static double xTarget = 0;
    public static double yTarget = 0;
    public static double thetaTarget = 0;

    public static double xP, xI, xD, xF = 0;

    public static double yP, yI, yD, yF = 0;

    public static double thetaP, thetaI, thetaD, thetaF = 0;



    PIDFController xControl = new PIDFController(xP,xI,xD,xF);
    PIDFController yControl = new PIDFController(yP,yI,yD,yF);
    PIDFController thetaControl = new PIDFController(thetaP,thetaI,thetaD,thetaF);

    @Override
    public void runOpMode() throws InterruptedException {

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

        MecanumDrive drive = new MecanumDrive(hardwareMap);



        waitForStart();
        while (opModeIsActive()) {
            xControl.setPIDF(xP, xI, xD, xF);

            yControl.setPIDF(yP, yI, yD, yF);

            thetaControl.setPIDF(thetaP, thetaI, thetaD, thetaF);




            Pose2d pose = drive.getPoseEstimate();
            double xRobotPosition = pose.getX();
            double yRobotPosition = pose.getY();
            double thetaRobotPosition = pose.getHeading();

            double x = xControl.calculate(xTarget, xRobotPosition);
            double y = yControl.calculate(yTarget, yRobotPosition);
            double t = thetaControl.calculate(thetaTarget, thetaRobotPosition);
            double x_rotated = x * Math.cos(thetaRobotPosition) - y * Math.sin(thetaRobotPosition);
            double y_rotated = x * Math.sin(thetaRobotPosition) + y * Math.cos(thetaRobotPosition);

            // x, y, theta input mixing
            frontLeftMotor.setPower(x_rotated + y_rotated + t);
            backLeftMotor.setPower(x_rotated - y_rotated + t);
            frontRightMotor.setPower(x_rotated - y_rotated - t);
            backRightMotor.setPower(x_rotated + y_rotated - t);

            telemetry.addData("xTarget", xTarget);
            telemetry.addData("yTarget", yTarget);
            telemetry.addData("thetaTarget", thetaTarget);

            telemetry.addData("xRobotPosition", xRobotPosition);
            telemetry.addData("yRobotPosition", yRobotPosition);
            telemetry.addData("thetaRobotPosition", thetaRobotPosition);

            telemetry.update();
        }
    }
}
