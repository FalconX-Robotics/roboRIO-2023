// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
  /** Creates a new Lift. */
  // Private variable for two neo motors
  private final CANSparkMax m_armRotation  = new CANSparkMax(Constants.ARM_ROTATION_MOTOR_PORT, MotorType.kBrushless);
  private final CANSparkMax m_armExtend = new CANSparkMax(Constants.ARM_EXTENSION_MOTOR_PORT, MotorType.kBrushless);
  
  private double m_targetAngle = 0;
  private double m_targetExtend = 0;
  private boolean m_isNewTarget = false;
  private ArmState m_currentArmState = ArmState.READY;

  public Lift() {
    
      // // GOING TO MOVE SOMETIME I GUESS
      // // Different states using moveToPosition
      // // Retracted State
      // moveToPosition(0,0);

      // // Ground Arm
      // moveToPosition(45,1);

      // // Mid Arm
      // moveToPosition(90,1);

      // // Human Intake
      // moveToPosition(100, 1);
  }

  public void moveToPosition(double angle, double extend) {
    m_targetAngle = angle;
    m_targetExtend = extend;
    m_isNewTarget = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

   private enum ArmState {
    READY,
    ROTATE_WAITING,
    ROTATING
   }

   private void updateArmState() {
      if (m_currentArmState == ArmState.READY)
      {
        if (m_isNewTarget)
        {
          m_currentArmState = ArmState.ROTATE_WAITING;
        }
      }
      else if (m_currentArmState == ArmState.ROTATE_WAITING)
      {
        if (isExtenderSafeForArm())
        {
          m_currentArmState = ArmState.ROTATING;
        }
      }
      else
      {
        if (m_isNewTarget)
        {
          m_currentArmState = ArmState.ROTATE_WAITING;
        }
        else if (inPosition())
        {
          m_currentArmState = ArmState.READY;
        }
      }
    }

  private boolean inPosition() {
    return false;
  }

  private boolean isExtenderSafeForArm() {
    return false;
  }
}
