package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ManualArm extends CommandBase {
    private double m_movementSpeed;
    private MovementType m_movementType;
    private Arm m_arm;

    public static enum MovementType {
        ROTATION,
        EXTENSION
    }

    public ManualArm(double speed, MovementType movement, Arm arm) {
        m_movementSpeed = speed;
        m_movementType = movement;
        m_arm = arm;

        addRequirements(arm);
    }
    
    @Override
    public void execute() {
        m_arm.manualMoveArm(m_movementType, 1);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
