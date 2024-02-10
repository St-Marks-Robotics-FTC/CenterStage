package org.firstinspires.ftc.teamcode.LM2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class LM2class {

    public DcMotorEx arm;
    public Servo clawLeft;
    public Servo clawRight;
    public Servo drone;

    public TouchSensor leftSensor;
    public TouchSensor rightSensor;

    private double target = 0.0;
    private int prev;

    public LM2class(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        //reversed on accident
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        drone = hardwareMap.get(Servo.class, "drone");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSensor = hardwareMap.get(TouchSensor.class, "leftSensor");
        rightSensor = hardwareMap.get(TouchSensor.class, "rightSensor");
    }

    public boolean clawState() {
        return target==0.3 ? true : false;
    }
    public void closeClaw() {
        closeLeft();
        closeRight();
        //target = 0.3;
    }

    public void openLeft() {
        clawLeft.setPosition(0.48);
    }
    public void scoreLeft() {
        clawLeft.setPosition(0.51);
    }
    public void closeLeft() {
        clawLeft.setPosition(0.6);
    }

    public void openRight() {
        clawRight.setPosition(0.5);
    }
    public void scoreRight() {
        clawRight.setPosition(0.57);
    }
    public void closeRight() {
        clawRight.setPosition(0.7);
    }




    public void openClaw() {
        openLeft();
        openRight();
        //target = 0.7;
    }

    public void scoreClaw() {
        scoreLeft();
        scoreRight();
        //target = 0.7;
    }

    public void closeDrone() {
        drone.setPosition(0.1);
    }

    public void openDrone() {
        drone.setPosition(0.5);
    }
    public void hang(int pos){
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(1);
    }

    public void safeRelease(int pos) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.7);
    }

    public void setArm(int pos) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower((pos>prev) ? 0.75 : 0.5);
        prev = pos;
    }

    public void zeroArm() {
        arm.setPower(-0.1);
    }

    public boolean detectLeft() {
        return leftSensor.isPressed();
    }
    public boolean detectRight() {
        return rightSensor.isPressed();
    }

}