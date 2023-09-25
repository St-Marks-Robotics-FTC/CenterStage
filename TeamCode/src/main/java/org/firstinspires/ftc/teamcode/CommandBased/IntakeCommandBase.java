package org.firstinspires.ftc.teamcode.CommandBased;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.function.BooleanSupplier;

public class IntakeCommandBase extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem m_intakeSubsystem;
    private final BooleanSupplier m_isPressed;
    public IntakeCommandBase(IntakeSubsystem subsystem, BooleanSupplier x) {
        m_intakeSubsystem = subsystem;
        m_isPressed = x;
        addRequirements(m_intakeSubsystem);
    }
    @Override
    public void execute(){
        if(m_isPressed.getAsBoolean()){
            m_intakeSubsystem.intake();
        }
        else{
            m_intakeSubsystem.stopIntake();
        }
    }
}
