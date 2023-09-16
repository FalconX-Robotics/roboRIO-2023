// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.annotation.processing.SupportedOptions;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ManualArm;
import frc.robot.util.BetterSlewRateLimiter;

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
 *
 * <h1> <em> c'est vrai
 * also you spelled information wrong (skill issue)
 * 
 * lorem ipsum dolor sit amet
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
        // julian knows
  private BetterSlewRateLimiter m_extendRateLimiter = new BetterSlewRateLimiter(1.5, 1.5, 0);
  private BetterSlewRateLimiter m_rotationRateLimiter = new BetterSlewRateLimiter(1.5, 5, 0);
  // Placeholders on gear ratio and radius; Change later
  private double armExtensionGearRatio = 72. / 11.;
  private double armExtensionGearRadius = 0.716; //in inches
  private double armRotationGearRatio = 5.0 / 1.0 * 52. / 18. * 58. / 18. * 64. / 15.;

  private boolean m_brakeModeOn = true;

  private boolean shouldMove = true;
  private boolean currentlyMoving = false;

  public Arm() {
    m_rotationArm.setInverted(false);
    m_extendArm.setInverted(false);

    setIdleMode(m_brakeModeOn);

    resetExtenderEncoder(0.5);
    resetRotationEncoder(32);

    m_rotationArm.setSmartCurrentLimit(40);
    m_extendArm.setSmartCurrentLimit(40);
  }

  public double getRotationArmPosition() {
    return m_rotationArm.getEncoder().getPosition() * 360 / armRotationGearRatio;
  }

  

  public double getExtensionArmPosition() {
    return m_extendArm.getEncoder().getPosition() * armExtensionGearRadius / armExtensionGearRatio * 2. * Math.PI;
  }

  public boolean unsafeMoveToPosition(double angle, double extend, double extensionLeniancy, double rotationLeniancy) {
    // System.out.println("Target angle: " + angle + " current Angle: " + getRotationArmPosition());
    setRotationMotor(MathUtil.clamp((angle - getRotationArmPosition()) * .02, -1., 1.));
    setExtensionMotor(MathUtil.clamp((extend - getExtensionArmPosition()) * .1, -.5, .5));
    
    // returns if the arm is in position
    return Math.abs(getRotationArmPosition() - angle) <= rotationLeniancy && Math.abs(getExtensionArmPosition() - extend) <= extensionLeniancy;
  }

  private void stopMotors() {
    setExtensionMotor(0);
    setRotationMotor(0);
  }

  // Moves arm to position using angle and extend (also stops it when needs to)
  public boolean moveToPosition(double angle, double extend) {
    if (Math.abs(angle - m_rotationArm.getEncoder().getPosition()) > 5) 
    {
      currentlyMoving = true;
      }
    // For testing, remove when avaliable.
    System.out.println("Changing angle to approximately " + Math.floor(angle) + " degrees");
    System.out.println("Moving extender to approximately " + Math.floor(extend) + " inches");

    m_targetAngle = angle;
    m_targetExtension = extend;
    m_isNewTarget = true;
    boolean armRotationCheck = false;
    boolean armExtensionCheck = false;
    if (!Constants.overrideDisable){
      if (getRotationArmPosition() > m_targetAngle + 5 ) {
        m_rotationArm.set(-0.2);
      } else if (getRotationArmPosition() < m_targetAngle - 5) {
        m_rotationArm.set(0.2);
      } else {
        m_rotationArm.set(0);
        armRotationCheck = true;
        currentlyMoving = false;
      }

      if (getExtensionArmPosition() > m_targetExtension + 1 ) {
        m_extendArm.set(-0.2);
      } else if (getExtensionArmPosition() < m_targetExtension - 1) {
        m_extendArm.set(0.2);
      } else {
        m_extendArm.set(0);
        armExtensionCheck = true;
      }
    }
    //moves the rotation to the target angle with 5 degrees of leniancy
    
    //moves the extender to the target length with 1 inches of leniancy

    
    return (armRotationCheck && armExtensionCheck);

    // this is really terrible -- logan
    // skill issue -w
  }

  public void setExtensionMotor(double percentOutput) {
    if (!Constants.overrideDisable) {
      double voltage = percentOutput * 12. - 0.3 * Math.cos(Math.toRadians(getRotationArmPosition()));
      m_extendArm.setVoltage(voltage);
      SmartDashboard.putNumber("extensionMotor", voltage);
    } else {
      m_extendArm.setVoltage(0);
      SmartDashboard.putNumber("extensionMotor", 0);
    }
  }
  public boolean moveToPosition2(double angle, double extend){

    setRotationMotor(MathUtil.clamp((getRotationArmPosition() - angle) * .02, -1, 1));
    setExtensionMotor(MathUtil.clamp((getExtensionArmPosition() - extend) * .1, -.3, .3));

    return Math.abs(getRotationArmPosition() - angle) <= 5 && Math.abs(getExtensionArmPosition() - extend) <= 1;
  }
  public void setRotationMotor(double percentOutput) {
    if (!Constants.overrideDisable){
      double extensionPercent = getExtensionArmPosition() / 17.;
      // inner peaces charge you with excitment
      double inVolts = .25;
      double outVolts = .4;

      double voltageFactor = inVolts * (1 - extensionPercent) + outVolts * extensionPercent;

      double voltage = percentOutput * 12 + voltageFactor * Math.sin(Math.toRadians(getRotationArmPosition()));
      m_rotationArm.setVoltage(voltage);
      SmartDashboard.putNumber("rotationMotor", voltage);
    } else {
      m_rotationArm.setVoltage(0);
      SmartDashboard.putNumber("rotationMotor", 0);
    }
  }

  public void manualMoveArm(double rotationSpeed, double extensionSpeed) {
    
    // currentlyMoving = rotationSpeed != 0;
    if (safeExtension()) {
      setRotationMotor(m_rotationRateLimiter.calculate(rotationSpeed) * .5);
      setExtensionMotor(m_extendRateLimiter.calculate(extensionSpeed) * .33);
    
    } else if (getExtensionArmPosition() > 0.25) {
      setExtensionMotor(m_extendRateLimiter.calculate(Math.min(extensionSpeed, 0)) * .33);
      setRotationMotor(m_rotationRateLimiter.calculate(Math.max(rotationSpeed, 0)) * .5);
    
    } else if (getRotationArmPosition() > 40 || rotationSpeed > 0) {
      setRotationMotor(m_rotationRateLimiter.calculate(rotationSpeed) * .5);
      setExtensionMotor(0);
    
    } else {
      setRotationMotor(m_rotationRateLimiter.calculate(rotationSpeed) * 0.1);
      setExtensionMotor(0);
    }
    
  }
  
// Strink cope = "fake";
// what is a strink -w
// The past tense of stronk -g
// oh of course, i just forgot thanks -w
// why not strunk -t
// That's the future tense of course -g
// what about the past particible -w
// In that case you would use 'strank' -g

  @Override
  public void periodic() {
    
    /*if (!currentlyMoving) {
      if (Math.abs(m_rotationArm.getEncoder().getVelocity()) > 2) {
        m_rotationArm.set(-0.01 * m_rotationArm.getEncoder().getVelocity());
      }
    }*/
    
    // Not correct right now, but main idea
    
    //System.out.println("extensionMotor " + m_extendArm.get());
 // System.out.println("rotationMotor " + m_rotationArm.get());
    // This method will be called once per scheduler run

    double extendMotor = getExtensionArmPosition();
    SmartDashboard.putNumber("extentionPosition", extendMotor);

    double rotateMotor = getRotationArmPosition();
    SmartDashboard.putNumber("rotationPosition", rotateMotor);
  }

  public void resetExtenderEncoder() {
    resetExtenderEncoder(0);
  }

  public void resetExtenderEncoder(double position) {
    m_extendArm.getEncoder().setPosition(position / armExtensionGearRadius * armExtensionGearRatio / 2. / Math.PI);;
  }

  public void resetRotationEncoder() {
    resetRotationEncoder(0);
  }

  public void resetRotationEncoder(double angle) {
    m_rotationArm.getEncoder().setPosition(angle / 360. * armRotationGearRatio);
  }
  
  private boolean safeExtension() {
    return getRotationArmPosition() > 230. || (getRotationArmPosition() < 130. && getRotationArmPosition() > 50.);
  }

  public void toggleBrakeMode() {
    m_brakeModeOn = !m_brakeModeOn;
    SmartDashboard.putBoolean("Brake Mode", m_brakeModeOn);
    setIdleMode(m_brakeModeOn);
  }

  private void setIdleMode(boolean brakeModeOn) {
    
    if (brakeModeOn) {
      
      m_extendArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
      m_rotationArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    } else {
      
      m_extendArm.setIdleMode(CANSparkMax.IdleMode.kCoast);
      m_rotationArm.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
  }

  
}//pollo - thomas