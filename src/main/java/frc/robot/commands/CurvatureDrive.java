// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.XboxController;

public class CurvatureDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  private final Drivetrain m_drivetrain;
  private final XboxController m_xboxController;
  
  public CurvatureDrive(Drivetrain drivetrain, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_xboxController = xboxController;

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_xboxController.setRumble(RumbleType.kBothRumble, 1);
    // speedForward is front and back motion
    m_drivetrain.turboModeOn = m_xboxController.getRightBumper();

    boolean quickTurn = m_xboxController.getLeftBumper();
    double speedForward = m_xboxController.getRightY() - m_xboxController.getRightTriggerAxis() + m_xboxController.getLeftTriggerAxis();
    // double rotateSpeed = m_xboxController.getLeftX();

    if (speedForward > 0 && !quickTurn) {
      m_drivetrain.curvatureDrive(speedForward, -m_xboxController.getLeftX() * (quickTurn?.325:.75), quickTurn);
    } else {
      m_drivetrain.curvatureDrive(speedForward, m_xboxController.getLeftX() * (quickTurn?.325:0.75), quickTurn);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_drivetrain.curvatureDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
