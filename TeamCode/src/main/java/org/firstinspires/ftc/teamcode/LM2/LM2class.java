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

    public void closeClaw() {
        closeLeft();
        closeRight();
        //target = 0.3;
    }

    public void openLeft() {
        clawLeft.setPosition(0.45);
    }
    public void scoreLeft() {
        clawLeft.setPosition(0.5);
    }
    public void closeLeft() {
        clawLeft.setPosition(0.6);
    }

    public void openRight() {
        clawRight.setPosition(0.44);
    }
    public void scoreRight() {
        clawRight.setPosition(0.36);
    }
    public void closeRight() {
        clawRight.setPosition(0.3);
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

    public void holdDrone() {
        drone.setPosition(0.1);
    }

    public void shootDrone() {
        drone.setPosition(0.5);
    }
    public void hang(int pos){
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(1);
    }



    public void setArm(int pos) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower((pos>prev) ? 0.75 : 0.5);
        prev = pos;
    }

    public boolean detectLeft() {
        //return false;
        return leftSensor.isPressed();
    }
    public boolean detectRight() {
        //return false;
        return rightSensor.isPressed();
    }

}