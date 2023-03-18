package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class TurboModeCommand extends CommandBase {
    private enum TurboModeType {
        HOLD,
        TOGGLE;
    }

    Drivetrain m_drivetrain;

    public TurboModeCommand(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    private TurboModeType type = TurboModeType.HOLD;

    @Override
    public void execute() {
        if (type == TurboModeType.HOLD) {
            Drivetrain.turboModeOn = false;    
        } 
        
    }

    @Override
    public void initialize() {
        if (type == TurboModeType.TOGGLE) {
            Drivetrain.turboModeOn = !Drivetrain.turboModeOn;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        if (type == TurboModeType.HOLD) {
            Drivetrain.turboModeOn = true;
        }
        
    }
}