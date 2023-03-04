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
    HIGH_ARM,
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
      case HIGH_ARM:
        highArmState();
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

  private static enum CurrentStep{
    RETRACTING,
    ROTATING,
    EXTENDING,
    FINISHED
    }

  CurrentStep m_armStep = CurrentStep.RETRACTING;

  private void moveByStep(double targetAngle, double extension){
    //if (<state 1>) moveToPostion(currentAngle, 0)
    //else if (<state 2>) moveToPostion(angle, 0)
    // else (state 3) moveToPostion(angle, extension)
    if(m_armStep == CurrentStep.RETRACTING){
      if (m_arm.moveToPosition(m_arm.getRotationArmPosition(), 0)){
        m_armStep = CurrentStep.ROTATING;
      }
    }else if(m_armStep == CurrentStep.ROTATING){
      if (m_arm.moveToPosition(targetAngle, 0)){
        m_armStep = CurrentStep.EXTENDING;
      }
    }else if (m_armStep == CurrentStep.EXTENDING) {
      if(m_arm.moveToPosition(targetAngle, extension)){
        m_armStep = CurrentStep.FINISHED;
      }
    }
  }

  // The different states for the arm
  // These values are not correct; need to change
  // The arm can extend a maximum of 17 inches
  // At maximum extension, arm is just below 28 inches in length
  private void retractedState() {
    System.out.println("Moving to retracted state");
    moveByStep(10, 0);
  }

  private void groundArmState() {
    System.out.println("Moving to ground arm state");
    moveByStep(45, 12);
  }
  
  private void midArmState() {
    System.out.println("Moving to mid arm state");
    moveByStep(90, 12);
  }

  private void highArmState() {
    System.out.println("Moving to high arm state");
    moveByStep(130, 12);
   

  }

  private void humanIntakeState() {
    System.out.println("Moving to human intake state");
    moveByStep(115,12);
  
    }

  private void movingState() {
    System.out.println("Moving to moveing state? idk");

    // whats this do????????
  }

  @Override
  public boolean isFinished() {
    return m_armStep == CurrentStep.FINISHED;
  }
}
