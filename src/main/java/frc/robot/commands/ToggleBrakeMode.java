package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ToggleBrakeMode extends CommandBase {
    Arm m_arm;

    public ToggleBrakeMode(Arm arm) {
        m_arm = arm;
    }

    @Override
    public void initialize() {
        m_arm.toggleBrakeMode();
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
