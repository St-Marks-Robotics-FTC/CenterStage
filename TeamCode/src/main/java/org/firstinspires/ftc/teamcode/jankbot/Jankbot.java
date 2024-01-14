package org.firstinspires.ftc.teamcode.jankbot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.jankbot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.jankbot.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.jankbot.Subsystems.SpecialTeams;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;

public class Jankbot {

    //public ActionQueue queue;
    public MecanumDrive drive;
    public Intake intake;
    public Outtake outtake;
    public SpecialTeams special;

    public Jankbot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
    }

//    public void update() {
//        intake.update();
//        outtake.update();
//    }

//    public void transfer() {
//        outtake.openBothClaw();
//        queue.addDelayedAction(()->intake.tiltUp(), 200);
//        queue.addDelayedAction(()-> outtake.v4barTransfer(), 600);
//        queue.addDelayedAction(()-> outtake.closeBothClaw(), 1000);
//        queue.addDelayedAction(()->intake.tiltDown(), 1500);
//    }
}
