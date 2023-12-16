package org.firstinspires.ftc.teamcode.jankbot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {

    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;
    public Servo v4barLeft;
    public Servo v4barRight;
    public Servo turret;
    public Servo pivot;
    public Servo clawLeft;
    public Servo clawRight;

    private int slideDown = 0;
    private int slideUp = 900;
    private int slideSlightUp=800;

    private double clawLeftClosed = 0.0;
    private double clawRightClosed = 0.0;
    private double clawLeftOpen = 0.0;
    private double clawRightOpen = 0.0;

    private int targetPosition = 0;
    private double slideDownPower=0.2;
    private double slideUpPower=0.5;

    private int TOLERANCE =10;

    public enum outtakeState {
        DEPOSIT,
        MOVING,
        TRANSFER
    }

    public outtakeState status;

    public Outtake (HardwareMap hardwareMap) {
        //outtake
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");

        pivot = hardwareMap.get(Servo.class, "pivot");
        turret = hardwareMap.get(Servo.class, "turret");
        v4barLeft = hardwareMap.get(Servo.class, "v4barLeft");
        v4barRight = hardwareMap.get(Servo.class, "v4barRight");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setTargetPositionTolerance(TOLERANCE);
        rightSlide.setTargetPositionTolerance(TOLERANCE);

        v4barRight.setDirection(Servo.Direction.REVERSE);

        status = outtakeState.TRANSFER;
    }

    public void update() {
        if (getSlidePos()<10) {
            status=outtakeState.TRANSFER;
        } else if (isDone()) {
            status=outtakeState.DEPOSIT;
        } else {
            status=outtakeState.MOVING;
        }
    }

    public void setSlide(int pos) {
        rightSlide.setTargetPosition(pos);
        leftSlide.setTargetPosition(pos);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(targetPosition>pos ? slideUpPower : slideDownPower);
        rightSlide.setPower(targetPosition>pos ? slideUpPower : slideDownPower);
        targetPosition=pos;
    }

    public boolean isDone() {
        return Math.abs(getSlidePos() - targetPosition)<TOLERANCE;
    }

    public int getSlidePos() {
        return (leftSlide.getCurrentPosition()+ rightSlide.getCurrentPosition())/2;
    }

    public void setPivot(double pos) {
        pivot.setPosition(pos);
    }

    public void setTurret(double pos) {
        turret.setPosition(pos);
    }

    public void setV4Bar(double pos) {
        v4barRight.setPosition(pos);
        v4barLeft.setPosition(pos);
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

}
