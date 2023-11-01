package org.firstinspires.ftc.teamcode.CommandBased;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class V4BarSubsystem extends SubsystemBase{
    private final Servo outtakeServoRight;
    private final Servo outtakeServoLeft;

    //top
    private final double r1 = 0.0;
    //middle
    private final double r2 = 0.0;
    //bottom
    private final double r3 = 0.0;

    private static double target = 0.0;

    public V4BarSubsystem(final HardwareMap hMap, final String OName, final String TName) {
        outtakeServoLeft = hMap.get(Servo.class, OName);
        outtakeServoRight = hMap.get(Servo.class, TName);
        setPos(r1);
    }

    public void switchPos(int pos) {
        switch (pos) {
            case 0:
                setPos(r1);
                break;
            case 1:
                setPos(r2);
                break;
            case 2:
                setPos(r3);
                break;
        }
    }

    public boolean done() {
        if (Math.abs(target-getPos())<=0.01) {
            return true;
        } else {
            return false;
        }
    }

    public double getPos() {
        return outtakeServoLeft.getPosition();
    }

    public void setPos(double pos) {
        target = pos;
        outtakeServoLeft.setPosition(pos);
        outtakeServoRight.setPosition(1.0-pos);
    }

}
