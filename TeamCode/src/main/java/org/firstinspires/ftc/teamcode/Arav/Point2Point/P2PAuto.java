package org.firstinspires.ftc.teamcode.Arav.Point2Point;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.MecanumDrive;

@Config
@Autonomous
public class P2PAuto extends LinearOpMode {

    public static double drivingTolerance = 4;

    public static double endpointTolerance = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        P2Pclass p2p = new P2Pclass();

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

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();
        while (opModeIsActive()) {

            while (p2p.getDistance(drive.getPoseEstimate()) > drivingTolerance && opModeIsActive()) {
                double powers[] = p2p.getPower(new Pose2d(40, 0, 0), drive.getPoseEstimate());
                drive.setMotorPowers(powers[0], powers[1], powers[3], powers[2]);

                telemetry.addData("xTarget", p2p.getTarget().getX());
                telemetry.addData("yTarget", p2p.getTarget().getY());
                telemetry.addData("thetaTarget", p2p.getTarget().getHeading());

                telemetry.addData("xRobotPosition", drive.getPoseEstimate().getX());
                telemetry.addData("yRobotPosition", drive.getPoseEstimate().getY());
                telemetry.addData("thetaRobotPosition", drive.getPoseEstimate().getHeading());

                telemetry.update();
            }



            while (p2p.getDistance(drive.getPoseEstimate()) > endpointTolerance && opModeIsActive()) {
                double powers[] = p2p.getPower(new Pose2d(40, 40, Math.toRadians(90)), drive.getPoseEstimate());
                drive.setMotorPowers(powers[0], powers[1], powers[3], powers[2]);

                telemetry.addData("xTarget", p2p.getTarget().getX());
                telemetry.addData("yTarget", p2p.getTarget().getY());
                telemetry.addData("thetaTarget", p2p.getTarget().getHeading());

                telemetry.addData("xRobotPosition", drive.getPoseEstimate().getX());
                telemetry.addData("yRobotPosition", drive.getPoseEstimate().getY());
                telemetry.addData("thetaRobotPosition", drive.getPoseEstimate().getHeading());

                telemetry.update();
            }




        }
    }
}
