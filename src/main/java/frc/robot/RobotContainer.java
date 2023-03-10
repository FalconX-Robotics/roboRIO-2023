// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.TimedDriveForward;
import frc.robot.commands.ToggleBrakeMode;
import frc.robot.commands.Autos;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ManualArm;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArmSequence;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.SlowModeCommand;
import frc.robot.commands.TankDrive;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  private final XboxController m_drivetrainController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
  private final XboxController m_armController = new XboxController(Constants.XBOX_CONTROLLER_PORT2);
  // private final Camera m_camera = new Camera();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final ManualArm m_manualArm = new ManualArm(m_armController, m_arm);
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(m_drivetrain, m_drivetrainController);
  private final TankDrive tankDrive = new TankDrive(m_drivetrain, m_drivetrainController);
  private final CurvatureDrive curvatureDrive = new CurvatureDrive(m_drivetrain, m_drivetrainController);
  Pneumatics pneumatics = new Pneumatics();
  private Command armUpCommand = Commands.startEnd(
    () -> {
      m_arm.setExtensionMotor(0);
      m_arm.setRotationMotor(.5);
    }, 
    () -> {m_arm.setExtensionMotor(0);
    m_arm.setRotationMotor(0);
  }, m_arm).withTimeout(.2);

  private Command yeetAuto = new SequentialCommandGroup(
    armUpCommand,
    new ClawCommand(pneumatics, true),
    new WaitCommand(2),
    new TimedDriveForward(m_drivetrain, -0.5, 2.0),
    new ClawCommand(pneumatics, false));

  private Command scoreAuto = new SequentialCommandGroup(
    new ClawCommand(pneumatics, true),
    new WaitCommand(0.5),
    new TimedDriveForward(m_drivetrain, 0.5, 0.5),
    new WaitCommand(1.0));

  
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

    // Command Scheduler that will help troubleshoot (hopefully)
    CommandScheduler.getInstance()
      .onCommandInitialize(
        command ->
          Shuffleboard.addEventMarker(
            "Command Initizalized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
      .onCommandInitialize(
        command ->
          Shuffleboard.addEventMarker(
            "Command Interrupted", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
      .onCommandInitialize(
        command ->
          Shuffleboard.addEventMarker(
            "Command Finished", command.getName(), EventImportance.kNormal));
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
    m_drivetrain.setDefaultCommand(curvatureDrive);
    m_arm.setDefaultCommand(m_manualArm);
  }

  private void configureButtonBindings() {
    
    // Main states for arm
    
    //XboxController IS ARM NOT MOVEMENT

    /* Just in case bumper retracted state in the case we don't use high arm
    Trigger rBumper = new JoystickButton(m_xboxController, XboxController.Button.kRightBumper.value);
    rBumper.onTrue(new MoveArm(m_arm, MoveArm.State.RETRACTED)); */

    // high
    Trigger yButton = new JoystickButton(m_armController, XboxController.Button.kY.value);
    yButton.onTrue(new MoveArmSequence(255., 16., m_arm).withTimeout(10.));
    // mid
    Trigger bButton = new JoystickButton(m_armController, XboxController.Button.kB.value);
    bButton.onTrue(new MoveArmSequence(260., 2., m_arm).withTimeout(10.));
    // low
    Trigger aButton = new JoystickButton(m_armController, XboxController.Button.kA.value);
    aButton.onTrue(new MoveArmSequence(290., 2., m_arm).withTimeout(10.));
    // go home and cry with the homies
    Trigger xButton = new JoystickButton(m_armController, XboxController.Button.kX.value);
    xButton.onTrue(new MoveArmSequence(45., 0.2, m_arm).withTimeout(10.));

    /*
    Trigger bButton = new JoystickButton(m_xboxController, XboxController.Button.kB.value);
    bButton.onTrue(new MoveArm(m_arm, MoveArm.State.GROUND_ARM));
*/
    Trigger leftBumper2 = new JoystickButton(m_armController, XboxController.Button.kLeftBumper.value);
    leftBumper2.onTrue(new ClawCommand(pneumatics, true));

    Trigger rightBumper2 = new JoystickButton(m_armController, XboxController.Button.kRightBumper.value);
    rightBumper2.onTrue(new ClawCommand(pneumatics, false));
    //XboxController2 IS MOVEMENT NOT ARM
    Trigger rightBumper = new JoystickButton(m_drivetrainController, XboxController.Button.kRightBumper.value);
    rightBumper.whileTrue(new SlowModeCommand());

    Trigger startButton2 = new JoystickButton(m_armController, XboxController.Button.kStart.value);
    startButton2.onTrue(new ResetEncoders(m_arm));

    
    Trigger backButton2 = new JoystickButton(m_armController, XboxController.Button.kBack.value);
    backButton2.onTrue(new ToggleBrakeMode(m_arm));
    
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
    // return new TimedDriveForward(m_drivetrain);
    return yeetAuto;
  }
}