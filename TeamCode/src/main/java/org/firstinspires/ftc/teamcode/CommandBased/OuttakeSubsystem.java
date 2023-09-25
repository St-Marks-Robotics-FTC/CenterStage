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

    public OuttakeSubsystem(final HardwareMap hMap, final String OName, final String TName) {
        outtakeServoOne = hMap.get(Servo.class, OName);
        outtakeServoTwo = hMap.get(Servo.class, TName);
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
