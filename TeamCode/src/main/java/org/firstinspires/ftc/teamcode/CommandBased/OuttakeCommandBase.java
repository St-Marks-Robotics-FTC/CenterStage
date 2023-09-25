package org.firstinspires.ftc.teamcode.CommandBased;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.function.BooleanSupplier;

public class OuttakeCommandBase extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final OuttakeSubsystem m_outtakeSubsystem;
    private final BooleanSupplier m_isPressed;
    public OuttakeCommandBase(OuttakeSubsystem subsystem, BooleanSupplier x) {
        m_outtakeSubsystem = subsystem;
        m_isPressed = x;
        addRequirements(m_outtakeSubsystem);
    }
    @Override
    public void execute(){
        if(m_isPressed.getAsBoolean()){
            m_outtakeSubsystem.outtake();
        }
        else{
            m_outtakeSubsystem.reset();
        }
    }
}
