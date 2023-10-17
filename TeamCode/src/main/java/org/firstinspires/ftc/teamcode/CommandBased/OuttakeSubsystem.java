package org.firstinspires.ftc.teamcode.CommandBased;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeSubsystem extends SubsystemBase{
    private final Servo outtakeServoOne;
    private final Servo outtakeServoTwo;
    private final Servo rotateServo;

    private final double r1 = 0.0;
    private final double r2 = 0.0;
    private final double r3 = 0.0;

    public OuttakeSubsystem(final HardwareMap hMap, final String OName, final String TName, final String RName) {
        outtakeServoOne = hMap.get(Servo.class, OName);
        outtakeServoTwo = hMap.get(Servo.class, TName);
        rotateServo = hMap.get(Servo.class, RName);
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

    public void outtake(){
        outtakeServoOne.setPosition(0.8);
        outtakeServoTwo.setPosition(0.8);
    }

    public void reset(){
        outtakeServoOne.setPosition(0);
        outtakeServoTwo.setPosition(0);
    }

}
