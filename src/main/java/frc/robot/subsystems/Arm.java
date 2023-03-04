// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ManualArm;

/**
 * <strong> Aperature Science </strong>
 * is the name for the subsystem containing
 * both the extender and the rotation motor.
 * -- william :)
 *
 * <p>shut up its an arm not aperature
 * -- logan
 * 
 * <h2>YES IT IS
 * - william :)
 * 
 * <h1> <em> false imformation
 */

public class Arm extends SubsystemBase {
  /** Creates a new Lift. */
  // Private variable for two neo motors
  private double deez = 69.420; // Julian was here
  //Julian was there
  private final CANSparkMax m_rotationArm  = new CANSparkMax(Constants.ARM_ROTATION_MOTOR_PORT, MotorType.kBrushless);
  private final CANSparkMax m_extendArm = new CANSparkMax(Constants.ARM_EXTENSION_MOTOR_PORT, MotorType.kBrushless);
      //Julian is everywhere...
  private double m_targetAngle = 0;
  private double m_targetExtension = 0;
  private boolean m_isNewTarget = false;
  private ArmState m_currentArmState = ArmState.READY;

  // Placeholders on gear ratio and radius; Change later
  private double armExtensionGearRatio = 5.0/1.0;
  private double armExtensionGearRadius = 0.477; //in inches
  private double armRotationGearRatio = 200.0/1.0;

  private boolean shouldMove = true;
  private boolean currentlyMoving = false;
  // public int[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][] EightHundredAndFiftyFiveDimensionalIntArray = {1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1};

  public Arm() {
    
    m_rotationArm.setInverted(false);
    m_extendArm.setInverted(false);

    m_rotationArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_extendArm.setIdleMode(CANSparkMax.IdleMode.kBrake);

  }

  public double getRotationArmPosition() {
    return m_rotationArm.getEncoder().getPosition() * 360 * armRotationGearRatio;
  }

  public double getExtensionArmPosition() {
    return m_extendArm.getEncoder().getPosition() * armExtensionGearRadius * armExtensionGearRatio * 2 * Math.PI;
  }

  // Moves arm to position using angle and extend (also stops it when needs to)
  public boolean moveToPosition(double angle, double extend) {
    if (Math.abs(angle - m_rotationArm.getEncoder().getPosition()) > 5) {currentlyMoving = true;}
    // For testing, remove when avaliable.
    System.out.println("Changing angle to approximately " + Math.floor(angle) + " degrees");
    System.out.println("Moving extender to approximately " + Math.floor(extend) + " inches");

    m_targetAngle = angle;
    m_targetExtension = extend;
    m_isNewTarget = true;
    boolean armRotationCheck = false;
    boolean armExtensionCheck = false;
    
    //moves the rotation to the target angle with 5 degrees of leniancy
    if (getRotationArmPosition() > m_targetAngle + 5 ) {
      m_rotationArm.set(-0.5);
    } else if (getRotationArmPosition() < m_targetAngle - 5) {
      m_rotationArm.set(0.5);
    } else {
      m_rotationArm.set(0);
      armRotationCheck = true;
      currentlyMoving = false;
    }
    //moves the extender to the target length with 1 inches of leniancy

    if (getExtensionArmPosition() > m_targetExtension + 1 ) {
      m_extendArm.set(-0.5);
    } else if (getExtensionArmPosition() < m_targetExtension - 1) {
      m_extendArm.set(0.5);
    } else {
      m_extendArm.set(0); 
      armExtensionCheck = true;
    }

    return armRotationCheck && armExtensionCheck;

    // this is really terrible -- logan
  }

  public void manualMoveArm(double rotationSpeed, double extensionSpeed) {
    currentlyMoving = rotationSpeed != 0;
    m_rotationArm.set(rotationSpeed * 0.1);
    m_extendArm.set(extensionSpeed * 0.1);
  }

  @Override
  public void periodic() {
    /*if (!currentlyMoving) {
      if (Math.abs(m_rotationArm.getEncoder().getVelocity()) > 2) {
        m_rotationArm.set(-0.01 * m_rotationArm.getEncoder().getVelocity());
      }
    }*/
    // This method will be called once per scheduler run
  }

  // Enum states for arm
  private enum ArmState {
    READY,
    ROTATE_WAITING,
    ROTATING
  }

  private void resetExtenderEncoder() {
    m_extendArm.getEncoder().setPosition(0);
  }
  private void resetExtenderEncoder(double position) {
    m_extendArm.getEncoder().setPosition(position);
  }
  /**
   * <h4>currently unused</h4>
   * Changes the current arm state to update its value
   * <p>ROTATE WAITING is when the arm has a new target but cannot move for some reason.
   * <p>ROTATING is when the arm is currently moving.
   * <p>READY is when the arm is in position and has no new target.
   */
  private void UpdateArmState() {
    switch (m_currentArmState) {
      case READY: 
        m_currentArmState = ArmState.ROTATE_WAITING;
        break;
      case ROTATE_WAITING: 
        m_currentArmState = ArmState.ROTATING;
        break;
      default:
        if (m_isNewTarget) {
          m_currentArmState = ArmState.ROTATE_WAITING;
        }
        else if (inPosition()) {
          m_currentArmState = ArmState.READY;
        }
        break;
    }
  }

  private boolean inPosition() {
    return false;
  }

  private boolean isExtenderSafeForArm() {
    return !(m_targetAngle < 0.2);
  }
}