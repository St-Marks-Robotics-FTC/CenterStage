package org.firstinspires.ftc.teamcode.Testing.CommandBased;
import com.arcrobotics.ftclib.command.CommandBase;

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
