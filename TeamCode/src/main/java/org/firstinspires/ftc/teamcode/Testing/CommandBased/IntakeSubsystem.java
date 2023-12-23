package org.firstinspires.ftc.teamcode.Testing.CommandBased;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase{
    private final DcMotor intakeMotor;

    private final double intakePower = 0.8;
    private final double outtakePower = -0.4;

    public IntakeSubsystem(final HardwareMap hMap, final String IName) {
        intakeMotor = hMap.dcMotor.get(IName);
    }

    public void intake(){
        intakeMotor.setPower(intakePower);
    }

    public void outtake() {
        intakeMotor.setPower(outtakePower);
    }

    public void stopIntake(){
        intakeMotor.setPower(0);
    }

}
