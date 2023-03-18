package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PIDSubsystem extends SubsystemBase {
    protected abstract double getMeasurement();
    protected abstract void useOutput(double output, double setpoint);
    private PIDController m_controller;

    PIDSubsystem(PIDController controller) {
        super();
        m_controller = controller;
    }
}