package org.firstinspires.ftc.teamcode.jankbot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.jankbot.util.ActionQueue;

@Config
public class Intake {

    public DcMotorEx intake;
    public Servo tilt1;
    public Servo tilt2;

    AnalogInput tiltAngle;


    public static double dropDown = 0.5;
    public static double dropUp = 0.0;


    public static double tiltUp = 0.6;
    public static double tiltDown =0.2;
    public static double tiltStow = 0.3;
    public static double tiltStack = 0.3;


    public Intake(HardwareMap hardwareMap) {
        //intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //transfer
        tilt1 = hardwareMap.get(Servo.class, "tilt1");
        tilt2 = hardwareMap.get(Servo.class, "tilt2");
        tilt2.setDirection(Servo.Direction.REVERSE);

        tiltAngle = hardwareMap.get(AnalogInput.class, "tiltAngle");
    }


    // Intake Motor
    public void setIntake(double power) {
        intake.setPower(power);
    }

    public double getPower() {
        return intake.getPower();
    }

    public double getVelocity() {
        return intake.getVelocity();
    }


    // Transfer Tilt
    public void tiltUp() {
        setTilt(tiltUp);
    }

    public void tiltDown() {
        setTilt(tiltDown);
    }

    public void tiltStow() {
        setTilt(tiltStow);
    }

    public void tiltStack() {
        setTilt(tiltStack);
    }

    public void setTilt(double pos) {
        tilt1.setPosition(pos);
        tilt2.setPosition(pos);
    }

    public double getTiltAngleDegrees() {
        return tiltAngle.getVoltage() / 3.3 * 360.0;
    }


}
