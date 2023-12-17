package org.firstinspires.ftc.teamcode.jankbot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.jankbot.util.ActionQueue;

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

    public intakeState status;
    private double targetPower;

    private ActionQueue queue;

    private int transferDelay = 1000;

    public Intake(HardwareMap hardwareMap) {
        //intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        dropdown = hardwareMap.get(Servo.class, "dropdown");
        //transfer
        transfer = hardwareMap.get(Servo.class, "transfer");
        transferClaw = hardwareMap.get(Servo.class, "transferClaw");

        queue = new ActionQueue();

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setPosition(transferDown);

        status = intakeState.IDLE;
        targetPower=0.0;
    }

    public void update() {
        if (status!=intakeState.TRANSFER) {
            if (targetPower>0) {
                status=intakeState.INTAKE;
            } else {
                status=intakeState.IDLE;
            }
        }

        queue.update();
    }

    public void resetQueue() {
        queue.clearQueue();
    }

    public void setIntake(double power) {
        intake.setPower(power);
        status = intakeState.INTAKE;
        targetPower=power;
    }

    public double getPower() {
        return targetPower;
    }

    public void transfer() {
        transfer.setPosition(transferPos);
        queue.addDelayedAction(() -> transferRelease(), transferDelay);
        status = intakeState.TRANSFER;
    }

    public void transferRelease() {
        transferClaw.setPosition(transferRelease);
    }
}
