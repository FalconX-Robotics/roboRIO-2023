// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.simulation.FiretruckSim;

/**
 * <strong> Aperature Science </strong>
 * is the name for the subsystem containing
 * both the extender and the rotation motor.
 * -- william :)
 *
 * <p>shut up its an arm not aperature
 * -- logan
 * 
 * <h1>YES IT IS
 * - william :)
 */

public class Arm extends SubsystemBase {
  /** Creates a new Lift. */
  // Private variable for two neo motors
  private double deez = 69.420; // Julian was here
  //Julian was there
  private CANSparkMax m_rotationArm  = new CANSparkMax(Constants.ARM_ROTATION_MOTOR_PORT, MotorType.kBrushless);
  private CANSparkMax m_extendArm = new CANSparkMax(Constants.ARM_EXTENSION_MOTOR_PORT, MotorType.kBrushless);
  private RelativeEncoder m_rotationEncoder;
  private RelativeEncoder m_extendEncoder;

  private double m_targetAngle = 0;
  private double m_targetExtend = 0;
  private boolean m_isNewTarget = false;
  private ArmState m_currentArmState = ArmState.READY;

  // Placeholders on gear ratio and radius; Change later
  private double armExtensionGearRatio = 1/1;
  private double armExtensionGearRadius = 0.477; //in inches
  private double armRotationGearRatio = 1/1;
  // public int[][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][][] EightHundredAndFiftyFiveDimensionalIntArray = {1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1}{1};

  public Arm() {
    
    m_rotationArm.setInverted(false);
    m_extendArm.setInverted(false);

    if (Robot.isReal()) {
      m_rotationEncoder = m_rotationArm.getEncoder();
      m_armEncoder = m_extendArm.getEncoder();
    } else {
      m_rotationEncoder = new RelativeEncoderSim("Rotation Encoder");
      m_extendEncoder = new RelativeEncoderSim("Extend Encoder");
    }
  }

  public double getRotationArmPosition() {
    return m_rotationEncoder.getPosition() * 360 * armRotationGearRatio;
  }

  public double getExtensionArmPosition() {
    return m_extendEncoder.getPosition() * armExtensionGearRadius * armExtensionGearRatio * 2 * Math.PI;
  }

  // Moves arm to position using angle and extend (also stops it when needs to)
  public void moveToPosition(double angle, double extend) {
    m_targetAngle = angle;
    m_targetExtend = extend;
    m_isNewTarget = true;

    if (getRotationArmPosition() > m_targetAngle + 5 ) {
      m_rotationArm.set(-0.1);
    } else if (getRotationArmPosition() < m_targetAngle - 5) {
      m_rotationArm.set(0.1);
    } else {
      m_rotationArm.set(0); 
    }

    if (getExtensionArmPosition() > m_targetExtend + 2 ) {
      m_extendArm.set(-0.1);
    } else if (getExtensionArmPosition() < m_targetExtend - 2) {
      m_extendArm.set( 0.1);
    } else {
      m_rotationArm.set(0); 
    }

    // this is really terrible -- logan
  }

  @Override
  public void periodic() {
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

  private void ArmState() {

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
    }// BEIJING EMBASSY was here

    }

  private boolean inPosition() {
    return false;
  }

  private boolean isExtenderSafeForArm() {
    return !(m_targetAngle < 0.2);
  }

  private FiretruckSim armSimulation;
  @Override
  public void simulationPeriodic() {
    armSimulation.setInput(m_rotationArm.get() * RobotController.getBatteryVoltage(), 
        m_extendArm.get() * RobotController.getBatteryVoltage());
    
    // builtin encoder on NEO
    m_rotationEncoder.setSimulationPosition(armSimulation.getArmAngleRads() / 2. / Math.PI * armRotationGearRatio);
    m_rotationEncoder.setSimulationVelocity(armSimulation.getArmAngleRadsPerSecond() / 2. / Math.PI * armRotationGearRatio);
    m_extendEncoder.setSimulationPosition(armSimulation.getExtenderDistanceMeters());
    m_extendEncoder.setSimulationVelocity(armSimulation.getExtendVelocityMetersPerSecond());
  }
}