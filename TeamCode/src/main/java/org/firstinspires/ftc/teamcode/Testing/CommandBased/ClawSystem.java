package org.firstinspires.ftc.teamcode.Testing.CommandBased;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSystem extends SubsystemBase{
    private final DcMotor clawMotor;
    private final Servo clawServo;

    public ClawSystem(final HardwareMap hMap) {
        clawMotor = hMap.dcMotor.get("clawMotor");
        clawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawServo = hMap.get(Servo.class, "clawServo");
    }

    public void drive(double y, double x, double rx, boolean slow) {
        clawMotor.setTargetPosition(0);
    }

}
