package org.firstinspires.ftc.teamcode.jankbot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;

public class jankbot {

    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;
    public DcMotorEx intake;
    public Servo dropdown;
    public Servo transfer;
    public Servo transferClaw;
    public Servo v4barLeft;
    public Servo v4barRight;
    public Servo turret;
    public Servo pivot;
    public Servo clawLeft;
    public Servo clawRight;
    public MecanumDrive drive;

    private double clawLeftClosed = 0.65;
    private double clawLeftOpen = 0.8;
    private double clawRightClosed = 0.34;
    private double clawRightOpen = 0.25;

    public enum outtakeState {
        UP,
        DOWN,
        TRANSFER
    }

    public enum intakeState {
        INTAKE,
        IDLE,
        TRANSFER
    }

    public jankbot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap);
        //outtake
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        pivot = hardwareMap.get(Servo.class, "pivot");
        turret = hardwareMap.get(Servo.class, "turret");
        v4barLeft = hardwareMap.get(Servo.class, "v4barLeft");
        v4barRight = hardwareMap.get(Servo.class, "v4barRight");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        //intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        dropdown = hardwareMap.get(Servo.class, "dropdown");
        //transfer
        transfer = hardwareMap.get(Servo.class, "transfer");
        transferClaw = hardwareMap.get(Servo.class, "transferClaw");
    }

    public void update() {
        intakeUpdate();
        outtakeUpdate();
    }

    public void intakeUpdate() {

    }

    public void outtakeUpdate() {

    }
    public void closeClaw() {
        closeLeft();
        closeRight();
    }
    public void openClaw() {
        openLeft();
        openRight();
        //target = 0.7;
    }
    public void closeLeft() {
        clawLeft.setPosition(clawLeftClosed);
    }
    public void closeRight() {
        clawRight.setPosition(clawRightClosed);
    }
    public void openLeft() {
        clawLeft.setPosition(clawLeftOpen);
    }
    public void openRight() {
        clawRight.setPosition(clawRightOpen);
    }

    public void setIntake(double power) {
        intake.setPower(power);
    }

}
