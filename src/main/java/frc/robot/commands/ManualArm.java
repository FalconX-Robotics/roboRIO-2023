package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ManualArm extends CommandBase {
    private double m_rotationSpeed;
    private double m_extensionSpeed;
    private Arm m_arm;

    public ManualArm(double rotationSpeed, double extensionSpeed, Arm arm) {
        m_rotationSpeed = rotationSpeed;
        m_extensionSpeed = extensionSpeed;
        m_arm = arm;

        addRequirements(arm);
    }
    
    @Override
    public void execute() {
        m_arm.manualMoveArm(m_rotationSpeed, m_extensionSpeed);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
