package org.firstinspires.ftc.teamcode.jankbot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SpecialTeams {

    DcMotorEx hang;
    Servo drone;

    private double hangPower=1.0;
    private double loadedPos=0.3;
    private double releasePos=0.5;

    public SpecialTeams (HardwareMap hardwareMap) {
        hang = hardwareMap.get(DcMotorEx.class, "hang");
        drone = hardwareMap.get(Servo.class, "drone");

        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setTargetPositionTolerance(10);
        drone.setPosition(loadedPos);
    }

    public void setHang(int pos) {
        hang.setTargetPosition(pos);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setPower(hangPower);
    }

    public int getHangPos() {
        return hang.getCurrentPosition();
    }

    public void releaseDrone() {
        drone.setPosition(releasePos);
    }
}
