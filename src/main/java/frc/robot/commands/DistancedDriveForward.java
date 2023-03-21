package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DistancedDriveForward extends CommandBase {

    Drivetrain m_drivetrain;
    double m_distance;
    double m_speed;

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        m_drivetrain.resetEncoders();
    }

    public DistancedDriveForward (Drivetrain drivetrain, double distance, double speed) {
        this.m_drivetrain = drivetrain;
        this.m_distance = distance;
        this.m_speed = speed;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        m_drivetrain.tankDrive(m_speed, m_speed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_distance) <= Math.abs(m_drivetrain.getDistance());
    }
    
}
