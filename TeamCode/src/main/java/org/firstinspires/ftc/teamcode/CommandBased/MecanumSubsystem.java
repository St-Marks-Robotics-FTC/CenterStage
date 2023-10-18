package org.firstinspires.ftc.teamcode.CommandBased;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumSubsystem extends SubsystemBase{
    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;

    public MecanumSubsystem(final HardwareMap hMap) {
        frontLeftMotor = hMap.dcMotor.get("frontLeft");
        backLeftMotor = hMap.dcMotor.get("backLeft");
        frontRightMotor = hMap.dcMotor.get("frontRight");
        backRightMotor = hMap.dcMotor.get("backRight");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void drive(double y, double x, double rx, boolean slow) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double powerMultiplier = slow ? 0.3 : 1.0;

        double frontLeftPower = (y + x + rx) / denominator * powerMultiplier;
        double backLeftPower = (y - x + rx) / denominator * powerMultiplier;
        double frontRightPower = (y - x - rx) / denominator * powerMultiplier;
        double backRightPower = (y + x - rx) / denominator * powerMultiplier;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

}
