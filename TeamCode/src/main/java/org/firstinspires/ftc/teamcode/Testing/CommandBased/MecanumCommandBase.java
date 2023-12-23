package org.firstinspires.ftc.teamcode.Testing.CommandBased;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class MecanumCommandBase extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final MecanumSubsystem m_mecanumSubsystem;
    private final DoubleSupplier m_y;
    private final DoubleSupplier m_x;
    private final DoubleSupplier m_rx;
    public MecanumCommandBase(MecanumSubsystem subsystem, DoubleSupplier y, DoubleSupplier x, DoubleSupplier rx) {
        m_mecanumSubsystem = subsystem;
        m_y = y;
        m_x = x;
        m_rx = rx;
        addRequirements(m_mecanumSubsystem);
    }
    @Override
    public void execute(){
        m_mecanumSubsystem.drive(m_y.getAsDouble(), m_x.getAsDouble(), m_rx.getAsDouble(), false);
    }
}
