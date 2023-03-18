package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ManualArm extends CommandBase {
    private double m_rotationSpeed;
    private double m_extensionSpeed;
    private double m_speedLimiter;
    private Arm m_arm;
    private XboxController m_controller;

    /**
     * Do not
     */
    public ManualArm(double rotationSpeed, double extensionSpeed, Arm arm) {
        m_rotationSpeed = rotationSpeed;
        m_extensionSpeed = extensionSpeed;
        m_arm = arm;

        addRequirements(arm);
    }
    
    public ManualArm(XboxController controller, double speedLimiter, Arm arm) {
        m_controller = controller;
        m_arm = arm;
        m_speedLimiter = speedLimiter;
        addRequirements(arm);
    }
    
    @Override
    public void execute() {
        m_rotationSpeed = m_controller.getLeftY() * m_speedLimiter;
        m_extensionSpeed = m_controller.getRightY() * m_speedLimiter;
        m_rotationSpeed = MathUtil.applyDeadband(m_rotationSpeed, 0.1);
        m_extensionSpeed = MathUtil.applyDeadband(m_extensionSpeed, 0.1);
        m_arm.manualMoveArm(m_rotationSpeed, m_extensionSpeed);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
