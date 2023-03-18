package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class TurboModeCommand extends CommandBase {

    Drivetrain m_drivetrain;

    public TurboModeCommand(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        m_drivetrain.turboModeOn = true;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.turboModeOn = false;
    }
}