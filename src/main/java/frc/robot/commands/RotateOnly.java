// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RotateOnly extends CommandBase {
  double m_rotation;
  double m_extension;
  double m_leniancy;
  double m_rotationLeniancy;
  Arm m_arm;
  boolean complete;

  /** Creates a new rotateOnly. */
  public RotateOnly(double rotation, double leniancy, Arm arm) {
    m_rotation = rotation;
    m_leniancy = leniancy;
    m_arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_extension = m_arm.getExtensionArmPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    complete = m_arm.unsafeMoveToPosition(m_rotation, m_extension, .25, m_leniancy);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return complete;
  }
}
