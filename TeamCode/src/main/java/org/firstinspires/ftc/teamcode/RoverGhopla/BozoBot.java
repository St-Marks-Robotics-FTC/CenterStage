package org.firstinspires.ftc.teamcode.RoverGhopla;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BozoBot {
    DcMotor arm;
    Servo leftclaw;
    Servo rightclaw;
    Servo drone;
    public BozoBot(HardwareMap hardwareMap){
        arm = hardwareMap.dcMotor.get("arm");
        leftclaw = hardwareMap.servo.get("clawLeft");
        rightclaw = hardwareMap.servo.get("clawRight");
        //drone = hardwareMap.servo.get("");


        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public void ArmUp(){
        arm.setTargetPosition(508);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
   }

    public void ArmDown(){
        arm.setTargetPosition(13);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
    }

    public void LeftOpen(){
        leftclaw.setPosition(0.48);
    }

    public void LeftClose(){
        leftclaw.setPosition(0.6);
    }

    public void RightOpen(){
        rightclaw.setPosition(0.46);
    }

    public void RightClose(){
        rightclaw.setPosition(0.3);
    }
}
