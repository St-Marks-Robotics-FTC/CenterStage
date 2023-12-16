package org.firstinspires.ftc.teamcode.jankbot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    public DcMotorEx intake;
    public Servo dropdown;
    public Servo transfer;
    public Servo transferClaw;

    private double clawLeftClosed = 0.65;
    private double clawLeftOpen = 0.8;
    private double clawRightClosed = 0.34;
    private double clawRightOpen = 0.25;

    private double transferDown=0.2;
    private double transferPos = 0.3;
    private double transferClose = 0.4;
    private double transferRelease = 0.5;

    public enum intakeState {
        INTAKE,
        IDLE,
        TRANSFER
    }

    public Intake(HardwareMap hardwareMap) {
        //intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        dropdown = hardwareMap.get(Servo.class, "dropdown");
        //transfer
        transfer = hardwareMap.get(Servo.class, "transfer");
        transferClaw = hardwareMap.get(Servo.class, "transferClaw");

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setPosition(transferDown);
    }

    public void update() {

    }

    public void setIntake(double power) {
        intake.setPower(power);
    }

    public void transfer() {
        transfer.setPosition(transferPos);
        transferRelease();
    }

    public void transferRelease() {
        transferClaw.setPosition(transferRelease);
    }
}
