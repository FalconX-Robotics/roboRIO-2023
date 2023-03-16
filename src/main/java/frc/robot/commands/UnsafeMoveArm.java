package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class UnsafeMoveArm extends CommandBase {
    
    private double m_angle;
    private double m_extend;
    private Arm m_arm;
    private boolean isDone;

    public UnsafeMoveArm(double targAngle, double targExtend, Arm arm){
        m_angle = targAngle;
        m_extend = targExtend;
        m_arm = arm;
        addRequirements(arm);
    }

    public static UnsafeMoveArm createRotateOnly(Arm arm, double targAngle){
        return new UnsafeMoveArm(targAngle, arm.getExtensionArmPosition(), arm);
    }

    public static UnsafeMoveArm createExtensionOnly(Arm arm, double targExtend){
        return new UnsafeMoveArm(arm.getRotationArmPosition(), targExtend, arm);
    }

    public void execute(){
        isDone = m_arm.unsafeMoveToPosition(m_angle, m_extend);
    }

    public boolean isFinished(){
        return isDone;
    }
}
