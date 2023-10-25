package org.firstinspires.ftc.teamcode.CommandBased;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
