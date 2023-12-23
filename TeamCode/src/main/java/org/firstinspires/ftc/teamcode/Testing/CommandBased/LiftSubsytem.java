package org.firstinspires.ftc.teamcode.Testing.CommandBased;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsytem extends SubsystemBase {

    private final DcMotorEx leftLift;
    private final DcMotorEx rightLift;


    public LiftSubsytem(final HardwareMap hMap, final String leftL, final String rightL) {
        leftLift = hMap.get(DcMotorEx.class, leftL);
        rightLift = hMap.get(DcMotorEx.class, rightL);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setTargetPositionTolerance(10);
        rightLift.setTargetPositionTolerance(10);
    }

    public void setPos(int pos) {
        leftLift.setTargetPosition(pos);
        rightLift.setTargetPosition(pos);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


}
