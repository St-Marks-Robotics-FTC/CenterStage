package org.firstinspires.ftc.teamcode.CommandBased;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SlideSubsystem extends SubsystemBase {

    public final DcMotorEx leftSlide;
    public final DcMotorEx rightSlide;

    private static double target = 0.0;

    public SlideSubsystem (final HardwareMap hMap, final String OName, final String TName) {
        leftSlide = hMap.get(DcMotorEx.class, OName);
        rightSlide = hMap.get(DcMotorEx.class, OName);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getPos() {
        return leftSlide.getCurrentPosition();
    }

    public boolean done() {
        if (Math.abs(target-getPos())<=10) {
            return true;
        } else {
            return false;
        }
    }

    public void setPos(int x) {
        leftSlide.setTargetPosition(x);
        rightSlide.setTargetPosition(x);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
