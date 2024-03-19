package org.firstinspires.ftc.teamcode.LebronBot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.LebronBot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.LebronBot.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.LebronBot.Subsystems.SpecialTeams;
import org.firstinspires.ftc.teamcode.LebronBot.Roadrunner.MecanumDrive;

public class LebronClass {

    public MecanumDrive drive;
    public Intake intake;
    public Outtake outtake;
    public SpecialTeams special;

    public LebronClass(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        special = new SpecialTeams(hardwareMap);
    }

}