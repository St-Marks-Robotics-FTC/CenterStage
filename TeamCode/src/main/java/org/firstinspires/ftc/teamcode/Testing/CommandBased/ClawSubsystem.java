package org.firstinspires.ftc.teamcode.Testing.CommandBased;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase{
    private final Servo outtakeServoLeft;
    private final Servo outtakeServoRight;
    private final Servo rotateServo;

    //swing left
    private final double r1 = 0.0;
    //middle
    private final double r2 = 0.0;
    //swing right
    private final double r3 = 0.0;

    private final double closed = 1.0;
    private final double open = 0.0;
    private static double target = 0.0;

    public ClawSubsystem(final HardwareMap hMap, final String OName, final String TName, final String RName) {
        outtakeServoLeft = hMap.get(Servo.class, OName);
        outtakeServoRight = hMap.get(Servo.class, TName);
        rotateServo = hMap.get(Servo.class, RName);
        setPos(open, 1);
    }

    public void rotate(double pos) {
        rotateServo.setPosition(pos);
    }

    public void switchRotate(int pos) {
        switch (pos) {
            case 0:
                rotate(r1);
                break;
            case 1:
                rotate(r2);
                break;
            case 2:
                rotate(r3);
                break;
        }
    }

    public void setPos(double pos, int yaw) {
        setPos(pos);
        switchRotate(yaw);
    }

    public boolean done() {
        if (Math.abs(target-getPos())<=0.05) {
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

    public void outtake(){
        setPos(open);
    }

    public void reset(){
        setPos(closed);
    }

}
