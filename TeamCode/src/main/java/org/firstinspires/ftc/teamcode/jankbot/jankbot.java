package org.firstinspires.ftc.teamcode.jankbot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.jankbot.hardware.Intake;
import org.firstinspires.ftc.teamcode.jankbot.hardware.Outtake;
import org.firstinspires.ftc.teamcode.jankbot.hardware.SpecialTeams;
import org.firstinspires.ftc.teamcode.jankbot.util.ActionQueue;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;

public class jankbot {

    public MecanumDrive drive;
    public Intake intake;
    public Outtake outtake;
    public SpecialTeams teams;
    private ActionQueue queue;

    private int transferDelay = 1000;

    public jankbot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        teams = new SpecialTeams(hardwareMap);
        queue = new ActionQueue();
    }

    public void update() {
        intake.update();
        outtake.update();
        queue.update();
    }

    public void transfer() {
        if (intake.status == Intake.intakeState.IDLE && outtake.status == Outtake.outtakeState.TRANSFER) {
            intake.transfer();
            queue.addDelayedAction(() -> intake.transfer(), transferDelay);
        }
    }
}
