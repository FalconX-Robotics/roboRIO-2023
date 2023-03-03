package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class ClawCommand extends CommandBase {

    Pneumatics pneumatics;
    XboxController xboxController;
    boolean clawOpen = false;
    
    public ClawCommand (Pneumatics pneumatics, XboxController xboxController) {
        this.pneumatics = pneumatics;
        this.xboxController = xboxController;
    }

    @Override
    public void initialize() {
        if (!clawOpen) {
            pneumatics.open();
        } else {
            pneumatics.close();
        }
        // clawOpen ? pneumatics.open() : pneumatics.close();
        // why ternary operator no work =(
    }
}
