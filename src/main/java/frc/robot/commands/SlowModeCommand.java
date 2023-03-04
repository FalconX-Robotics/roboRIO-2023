package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class SlowModeCommand extends CommandBase {
    private enum SlowModeType {
        HOLD,
        TOGGLE;
    }

    private SlowModeType type = SlowModeType.HOLD;

    @Override
    public void execute() {
        if (type == SlowModeType.HOLD) {
            Drivetrain.slowModeOn = true;    
        } 
        
    }

    @Override
    public void initialize() {
        if (type == SlowModeType.TOGGLE) {
            Drivetrain.slowModeOn = !Drivetrain.slowModeOn;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        if (type == SlowModeType.HOLD) {
            Drivetrain.slowModeOn = false;
        }
        
    }
}