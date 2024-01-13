package org.firstinspires.ftc.teamcode.jankbot2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class jankbot2Class {

    public DcMotorEx arm;
    public Servo clawLeft;
    public Servo clawRight;
    public Servo autoClaw;

    private double target = 0.0;
    private int prev;

    public jankbot2Class(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        //autoClaw = hardwareMap.get(Servo.class, "autoClaw");

//        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean clawState() {
        return target==0.3 ? true : false;
    }
    public void closeClaw() {
        closeLeft();
        closeRight();
        //target = 0.3;
    }
    public void closeLeft() {
        clawLeft.setPosition(0.65);
    }
    public void closeRight() {
        clawRight.setPosition(0.34);
    }
    public void openLeft() {
        clawLeft.setPosition(0.8);
    }
    public void openRight() {
        clawRight.setPosition(0.25);
    }

    public void openClaw() {
        openLeft();
        openRight();
        //target = 0.7;
    }
//    public void hang(int pos){
//        arm.setTargetPosition(pos);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setPower(1);
//    }

//    public void safeRelease(int pos) {
//        arm.setTargetPosition(pos);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setPower(0.7);
//    }

    public void setArm(int pos) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower((pos>prev) ? 0.75 : 0.5);
        prev = pos;
    }

    public void zeroLift() {
        arm.setPower(-0.1);
    }

}