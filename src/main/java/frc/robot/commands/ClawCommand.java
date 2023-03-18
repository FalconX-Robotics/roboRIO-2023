package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class ClawCommand extends CommandBase {

    Pneumatics pneumatics;
    boolean openClaw;
    
    public ClawCommand (Pneumatics pneumatics, boolean open) {
        this.pneumatics = pneumatics;
        this.openClaw = open;
        addRequirements(pneumatics);
        System.out.println("CLAW, innit");
    }

    @Override
    public void initialize() {
        if (openClaw) {
            pneumatics.open();
            System.out.println("CLAW opens");
        } else {
            pneumatics.close();
            System.out.println("CLAW closes");
        }
        // clawOpen ? pneumatics.open() : pneumatics.close();
        // why ternary operator no work =(
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
