package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends PIDCommand {
    // public AutoBalance (Drivetrain drivetrain) {}
  private final PIDController m_controller;
  private DoubleSupplier m_measurement;
  private DoubleSupplier m_setpoint;
  private DoubleConsumer m_useOutput;

    Drivetrain m_drivetrain;
    PigeonIMU m_gyro;

    public AutoBalance(Drivetrain drivetrain) {
        super(m_controller, m_setpoint, m_measurement, m_useOutput, null)
        // Will start auto for 3 seconds and then turn off
        m_drivetrain = drivetrain;
        m_gyro = drivetrain.getGyro();
        addRequirements(m_drivetrain);
    }

}
