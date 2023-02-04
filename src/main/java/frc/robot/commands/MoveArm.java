// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */

  // Instance variables
  private double m_angle;
  private double m_extend;
  
  private Arm m_arm;

  // Different enum states/positions for arm
  public static enum State {
    RETRACTED, 
    GROUND_ARM, 
    MID_ARM, 
    HUMAN_INTAKE, 
    MOVING
  };

  State state;
  /** <h2> Literally useless, </h2> @return {@code true}
  */
  private boolean Joke () {
    boolean value = true;
    if (value == true) {
      return true;
    } else {
      return false;
    }
  }

  public MoveArm(Arm arm, State state) {
    // Use addRequirement() here to declare subsystem dependencies.
    // Set our parameters equal to the instance variables
    // TODO change later to match
    addRequirements(arm);
    m_arm = arm;
    this.state = state;
  }

  private void MoveArmToState (State state) {
    
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state) {
      case RETRACTED:
        retractedState();
        break;
      case GROUND_ARM:
        groundArmState();
        break;
      case MID_ARM:
        midArmState();
        break;
      case HUMAN_INTAKE:
        humanIntakeState();
        break;
      case MOVING:
        movingState();
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // The different states for the arm
  private void retractedState() {
    System.out.println("Moving to retracted state");
    if (m_arm.moveToPosition(10, 2)) {
      end(false); 
    }
  }

  private void groundArmState() {
    System.out.println("Moving to ground arm state");
    if (m_arm.moveToPosition(45, 12)) {
      end(false);
    }
  }
  
  private void midArmState() {
    System.out.println("Moving to mid arm state");
    if (m_arm.moveToPosition(90, 12)) {
      end(false);
    }
  }

  private void humanIntakeState() {
    System.out.println("Moving to human intake state");
    if (m_arm.moveToPosition(115, 12)) {
      end(false);
    }
  }

  private void movingState() {
    System.out.println("Moving to moveing state? idk");

    // whats this do????????
  }
}
