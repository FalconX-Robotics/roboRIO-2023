package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {

    boolean clawOpen = false;
    
    private final Compressor m_compressor = new Compressor(20, PneumaticsModuleType.CTREPCM);
    
    private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(20, PneumaticsModuleType.CTREPCM, 1, 0);
    public void open() {
        m_doubleSolenoid.set(Value.kForward);
    }
    public void close() {
        m_doubleSolenoid.set(Value.kReverse);
    }

    @Override
    public final void periodic() {
        SmartDashboard.putBoolean("Pressure Switch On", m_compressor.getPressureSwitchValue());
    }
}

