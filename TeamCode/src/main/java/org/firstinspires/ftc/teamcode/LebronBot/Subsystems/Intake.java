package org.firstinspires.ftc.teamcode.LebronBot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

//import org.firstinspires.ftc.teamcode.jankbot.util.ActionQueue;

@Config
public class Intake {

    public DcMotorEx intake;
    public Servo tilt1;
    public Servo tilt2;

    TouchSensor pixel1;
    TouchSensor pixel2;
    TouchSensor alignSwitch;
    AnalogInput tiltAngle;


    public static double dropDown = 0.5;
    public static double dropUp = 0.0;


    public static double tiltUp = 0.45; // .7
    public static double tiltUpDegrees = 150;
    public static double tiltDown =0.15;
    public static double tiltStow = 0.41;
    public static double tiltStack = 0.255;
    public static double tiltStackInc = 0.03;
    public static double offset = 0;

    public Intake(HardwareMap hardwareMap) {
        //intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        //transfer
        tilt1 = hardwareMap.get(Servo.class, "tiltLeft");
        tilt2 = hardwareMap.get(Servo.class, "tiltRight");
        tilt2.setDirection(Servo.Direction.REVERSE);

//        tiltAngle = hardwareMap.get(AnalogInput.class, "tiltAngle");
//
//
//        pixel1 = hardwareMap.get(TouchSensor.class, "pixel1");
//        pixel2 = hardwareMap.get(TouchSensor.class, "pixel2");
//        alignSwitch = hardwareMap.get(TouchSensor.class, "alignSwitch");
    }

    public void setOffset(double pos) {
        offset = pos;
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

    public void tiltStackTo(int pos) {
        //0 is highest
        setTilt(tiltStack-pos*tiltStackInc);
    }

    public void setTilt(double pos) {
        tilt1.setPosition(pos+offset);
        tilt2.setPosition(pos+offset);
    }

    // Sensors
    public boolean is2Aligned() {
        return getPixel1() && getPixel2() && isPixelAligned();
    }

    public boolean is2NotAligned() {
        return getPixel1() && getPixel2() && !isPixelAligned();
    }

    public boolean getPixel1() {
        return pixel1.isPressed();
    }
    public boolean getPixel2() {
        return pixel2.isPressed();
    }

    public boolean isPixelAligned() {
        return alignSwitch.isPressed();
    }

    public boolean isTiltUp() {
        return getTiltAngleDegrees() >= tiltUpDegrees;
    }

    public double getTiltAngleDegrees() {
        return tiltAngle.getVoltage() / 3.3 * 360.0;
    }

    public void setStack(int level) {
        if (level == 1) {
            tiltDown();
        } else if (level == 2) {
            setTilt(0.184);
        } else if (level == 3) {
            setTilt(0.196);
        } else if (level == 4) {
            setTilt(0.213);
        } else if (level == 5) {
            setTilt(0.24);
        } else {
            tiltDown();
        }
    }

}