package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class UnsafeMoveArm extends CommandBase {
    
    private double m_angle;
    private double m_extend;
    private double m_extendLeniency;
    private double m_rotationLeniancy;
    private Arm m_arm;
    private boolean isDone;

    public UnsafeMoveArm(double targAngle, double targExtend, double extendLeniency, double rotationLeniancy, Arm arm){
        m_angle = targAngle;
        m_extend = targExtend;
        m_rotationLeniancy = rotationLeniancy;
        m_extendLeniency = extendLeniency;
        m_arm = arm;
        addRequirements(arm);
    }

    // public static UnsafeMoveArm createRotateOnly(Arm arm, double targAngle){
    //     return new UnsafeMoveArm(targAngle, arm.getExtensionArmPosition(), arm);
    // }

    // public static UnsafeMoveArm createExtensionOnly(Arm arm, double targExtend){
    //     return new UnsafeMoveArm(arm.getRotationArmPosition(), targExtend, arm);
    // }

    public void execute(){
        isDone = m_arm.unsafeMoveToPosition(m_angle, m_extend, m_extendLeniency, m_rotationLeniancy);
    }

    public boolean isFinished(){
        return isDone;
    }
}
