package org.firstinspires.ftc.teamcode.Testing.CommandBased;
import com.arcrobotics.ftclib.command.CommandBase;

public class Outtake extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ClawSubsystem m_clawSubsystem;
    private final V4BarSubsystem m_v4barSubsystem;

    private enum OuttakeState {
        RAISING,
        MOVING_ARM,
        DROPPING,
        RETRACTING,
        IDLE
    }

    public OuttakeState state = OuttakeState.IDLE;
    public Outtake(ClawSubsystem claw, V4BarSubsystem v4bar) {
        m_clawSubsystem = claw;
        m_v4barSubsystem = v4bar;
        addRequirements(m_clawSubsystem, m_v4barSubsystem);
    }
    @Override
    public void execute(){
        switch (state) {
            case IDLE:
                break;
        }
    }
}
