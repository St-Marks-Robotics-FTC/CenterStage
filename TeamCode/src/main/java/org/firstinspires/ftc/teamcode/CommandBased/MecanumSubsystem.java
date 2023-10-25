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

    public MecanumSubsystem(final HardwareMap hMap, final String FLName, final String BLName, final String FRName, final String BRName) {
        frontLeftMotor = hMap.dcMotor.get(FLName);
        backLeftMotor = hMap.dcMotor.get(BLName);
        frontRightMotor = hMap.dcMotor.get(FRName);
        backRightMotor = hMap.dcMotor.get(BRName);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double y, double x, double rx){
//        if(Math.abs(y)<=0.15 && Math.abs(x)<=0.15 && Math.abs(rx)<=0.15){
//            brake();
//        }
//        else {
            if(Math.abs(y)<=0.15){
                y=0;
            }
            if(Math.abs(x)<=0.15){
                x=0;
            }
            if(Math.abs(rx)<=0.15){
                rx=0;
            }
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
//            if(frontLeftPower==0 && frontRightPower == 0 && backRightPower == 0 && backLeftPower == 0){
//                brake();
//            }
//            else {
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            //}
        //}
    }
//    public void brake(){
//
//    }

}
