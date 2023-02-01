// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import /src/main/java/frc/robot/subsystems/Lift.java;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */

  // Instance variables
  private double m_angle;
  private double m_extend;
  private final XboxController m_xboxController;


  public MoveArm(double angle, double extend, XboxController xboxController) {
    // Use addRequirement() here to declare subsystem dependencies.
    // Set our parameters equal to the instance variables
    m_angle = angle;
    m_extend = extend;
    m_xboxController = xboxController;
    
    addRequirements(m_xboxController);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Not sure how values will be implemented yet, so leaving those spaces open for now
    if(m_xboxController.getAButtonPressed()){
      List.moveToPosition();
    }else if(m_xboxController.getBButtonPressed()){
      List.moveToPosition();
    }else if(m_xboxController.getXButtonPressed()){
      List.moveToPosition();
    }else if(m_xboxController.getYButtonPressed()){
      List.moveToPosition();
    }
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
