package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class ResetEncoders extends CommandBase{
    Arm m_arm;

    public ResetEncoders(Arm arm) {
        m_arm = arm;
    }

    @Override
    public void initialize() {
        m_arm.resetExtenderEncoder(0.5);
        m_arm.resetRotationEncoder(32);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
