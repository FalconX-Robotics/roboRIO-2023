// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class Autonomous extends CommandBase {
  /** Creates a new Autonomous. */

  private Drivetrain m_drivetrain;
  private double startTime;
  
  
  public Autonomous() {
    // Will start auto for 3 seconds and then turn off
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.tankDrive(0.5, 0.5);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - startTime >= 3.0) {
        return true;
    } else {
      return false;
    }
  }
}
