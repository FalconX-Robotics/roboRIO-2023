package frc.robot.simulation;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class MotorControllerSim implements MotorController {

    double m_percentOutput = 0.0;
    boolean m_isInverted = false;
    String m_name;

    public MotorControllerSim(String name) {
        m_name = name;
    }

    @Override
    public void set(double speed) {
        m_percentOutput = speed;
    }

    @Override
    public double get() {
        return m_percentOutput * (m_isInverted ? -1 : 1);
    }

    @Override
    public void setInverted(boolean isInverted) {
        m_isInverted = isInverted;
    }

    @Override
    public boolean getInverted() {
        return m_isInverted;
    }

    @Override
    public void disable() {
    }

    @Override
    public void stopMotor() {
        m_percentOutput = 0.0;
    }
    
}
