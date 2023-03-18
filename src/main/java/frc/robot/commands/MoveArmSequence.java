package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;


public class MoveArmSequence extends SequentialCommandGroup{
    
    public MoveArmSequence(double targetAngle, double targetExtend, Arm arm){
        super(
            // UnsafeMoveArm.createExtensionOnly(arm, 0.25),
            // UnsafeMoveArm.createRotateOnly(arm, targetAngle),
            // UnsafeMoveArm.createExtensionOnly(arm, targetExtend)
            new ExtendOnly(0.25, 1., arm),
            new RotateOnly(targetAngle, 10., arm),
            new UnsafeMoveArm(targetAngle, targetExtend, .25, 1., arm)
        );
    }
}