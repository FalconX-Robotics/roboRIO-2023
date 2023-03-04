// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.TimedDriveForward;
import frc.robot.commands.Autos;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ManualArm;
import frc.robot.commands.MoveArm;
import frc.robot.commands.SlowModeCommand;
import frc.robot.commands.TankDrive;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final XboxController m_xboxController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
  // private final Camera m_camera = new Camera();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(m_drivetrain, m_xboxController);
  private final TankDrive tankDrive = new TankDrive(m_drivetrain, m_xboxController);
  private final CurvatureDrive curvatureDrive = new CurvatureDrive(m_drivetrain, m_xboxController);
  Pneumatics pneumatics = new Pneumatics();
  // private final AutoBalance autoBalance = new AutoBalance(m_drivetrain);

  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
     // new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
   
  private void configureBindings() {
    configureButtonBindings();
    m_drivetrain.setDefaultCommand(arcadeDrive);
  }

  private void configureButtonBindings() {
    
    // Main states for arm
    Trigger rBumper = new JoystickButton(m_xboxController, XboxController.Button.kRightBumper.value);
    rBumper.onTrue(new MoveArm(m_arm, MoveArm.State.RETRACTED));

    Trigger aButton = new JoystickButton(m_xboxController, XboxController.Button.kA.value);
    aButton.onTrue(new MoveArm(m_arm, MoveArm.State.GROUND_ARM));

    Trigger xButton = new JoystickButton(m_xboxController, XboxController.Button.kX.value);
    xButton.onTrue(new MoveArm(m_arm, MoveArm.State.MID_ARM));

    Trigger yButton = new JoystickButton(m_xboxController, XboxController.Button.kX.value);
    yButton.onTrue(new MoveArm(m_arm, MoveArm.State.HIGH_ARM));  
    
    Trigger bButton = new JoystickButton(m_xboxController, XboxController.Button.kB.value);
    bButton.onTrue(new MoveArm(m_arm, MoveArm.State.HUMAN_INTAKE));


    Trigger rTrigger = new Trigger(() -> {
      return m_xboxController.getRightTriggerAxis() > 0.3;
    });
    
    rTrigger.whileTrue(new MoveArm(m_arm, MoveArm.State.HUMAN_INTAKE));

    // Trigger aButton = new JoystickButton(m_xboxController, XboxController.Button.kRightBumper.value);
    // aButton.whileTrue(new SlowModeCommand());
    // Trigger bButton = new JoystickButton(m_xboxController, XboxController.Button.kRightBumper.value);
    // bButton.onTrue(new ClawCommand(pneumatics, m_xboxController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    // return new AutoBalance(0, m_drivetrain);
    return new TimedDriveForward(m_drivetrain);
  }
}