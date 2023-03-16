package frc.robot.simulation;

import com.revrobotics.CANSparkMax;

public class CANSparkMaxSim extends CANSparkMax {

    double m_percentOutput = 0.0;
    boolean m_isInverted = false;
    String m_name;

    public CANSparkMaxSim(String name, int id, MotorType motorType) {
        super(id, motorType);
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
