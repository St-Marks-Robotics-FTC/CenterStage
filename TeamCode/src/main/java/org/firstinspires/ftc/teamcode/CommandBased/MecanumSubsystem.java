package org.firstinspires.ftc.teamcode.CommandBased;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;

public class MecanumSubsystem extends SubsystemBase{

    MecanumDrive mecanumDrive;

    public MecanumSubsystem(final HardwareMap hMap) {

        mecanumDrive = new MecanumDrive(hMap);

    }

    public void drive(double y, double x, double rx, boolean slow) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double powerMultiplier = slow ? 0.3 : 1.0;
        if(Math.abs(y)<=0.15){
            y=0;
        }
        if(Math.abs(x)<=0.15){
            x=0;
        }
        if(Math.abs(rx)<=0.15){
            rx=0;
        }
        double frontLeftPower = (y + x + rx) / denominator * powerMultiplier;
        double backLeftPower = (y - x + rx) / denominator * powerMultiplier;
        double frontRightPower = (y - x - rx) / denominator * powerMultiplier;
        double backRightPower = (y + x - rx) / denominator * powerMultiplier;

        mecanumDrive.setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
    }

}
