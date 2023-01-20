// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Autonomous extends CommandBase {
  /** Creates a new Autonomous. */

  private double time = Timer.getFPGATimestamp();
  private Drivetrain m_drivetrain;
  private double startTime;
  
  
  public Autonomous() {
    // Will start auto for 3 seconds and then turn off
    while (time - startTime <= 3)
    {
      m_drivetrain.tankDrive(0.5, 0.5);
      time = Timer.getFPGATimestamp();
    }
    m_drivetrain.tankDrive(0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      startTime = Timer.getFPGATimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
