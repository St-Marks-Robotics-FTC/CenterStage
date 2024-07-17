package org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.LebronBot.LebronClass;
import org.firstinspires.ftc.teamcode.LebronBot.PurePursuit.RobotMovement;

@Autonomous
public class PositionHold extends LinearOpMode {
    private LebronClass robot;
    private Pose2d target = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new LebronClass(hardwareMap);
        robot.drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            RobotMovement.goTO(robot.drive, target, 1, 1);
            RobotMovement.update(robot.drive);
        }
    }
}
