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
import frc.robot.commands.DistancedDriveForward;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ManualArm;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArmSequence;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.TurboModeCommand;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoBalance;;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static final String m_yeetAutoString = "YeetAuto";
  private static final String m_scoreAutoString = "ScoreAuto";
  private static final String m_autoBalanceString = "BalanceAuto";
  private static final String m_balanceNoMobilityString = "BalanceNoMobility";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final XboxController m_drivetrainController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
  private final XboxController m_armController = new XboxController(Constants.XBOX_CONTROLLER_PORT2);
  // private final Camera m_camera = new Camera();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final ManualArm m_manualArm = new ManualArm(m_armController, 0.5, m_arm);
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(m_drivetrain, m_drivetrainController);
  private final TankDrive tankDrive = new TankDrive(m_drivetrain, m_drivetrainController);
  private final CurvatureDrive curvatureDrive = new CurvatureDrive(m_drivetrain, m_drivetrainController, m_arm);
  Pneumatics pneumatics = new Pneumatics();
  private Command armUpCommand = Commands.startEnd(
    () -> {
      m_arm.setExtensionMotor(0);
      m_arm.setRotationMotor(.5);
    }, 
    () -> {m_arm.setExtensionMotor(0);
    m_arm.setRotationMotor(0);
  }, m_arm).withTimeout(.2);

  private Command armUpCommand2 = Commands.startEnd(
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
      armUpCommand2,
      new ClawCommand(pneumatics, true)
  );

  private Command balanceAuto = new SequentialCommandGroup(
    scoreAuto,
    new DistancedDriveForward(m_drivetrain, -3.7, -0.25),
    new WaitCommand(2),
    new AutoBalance(m_drivetrain, -1)
  );

  private Command balanceNoMobilityAuto = new SequentialCommandGroup(
    // scoreAuto,
    new WaitCommand(1),
    new AutoBalance(m_drivetrain)
  );

  
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

    // Camera code
    CameraServer.startAutomaticCapture();

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

    // Sendable Chooser stuff
    m_chooser.setDefaultOption("Yeet Auto", m_yeetAutoString);
    m_chooser.addOption("Score Auto", m_scoreAutoString);
    m_chooser.addOption("Auto Balance", m_autoBalanceString);
    m_chooser.addOption("Auto Balance no Mobility", m_balanceNoMobilityString);
    SmartDashboard.putData("Auto Selecter", m_chooser);

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

    Trigger joystickInterrupt = new Trigger(() -> {
      // CommandScheduler.getInstance().
      return m_armController.getRightY() >  .2 ||
             m_armController.getLeftY()  >  .2 ||
             m_armController.getRightY() < -.2 ||
             m_armController.getLeftY()  < -.2;
            //  !m_arm.getCurrentCommand().; 
    });
    // This is stupid but it should probably properly interrupt? -w
    // joystickInterrupt.onTrue(new MoveArmSequence(m_arm.getRotationArmPosition(), m_arm.getRotationArmPosition(), m_arm).withTimeout(0.));
    
    // Please confirm this before using as a reference -w
    //   Y
    // X   B
    //   A

    // high
    Trigger yButton = new JoystickButton(m_armController, XboxController.Button.kY.value);
    yButton.onTrue(new MoveArmSequence(250., 16.5, m_arm)
    .withTimeout(10.)
    .until(joystickInterrupt));

    // mid
    Trigger bButton = new JoystickButton(m_armController, XboxController.Button.kB.value);
    bButton.onTrue(new MoveArmSequence(260., 1., m_arm)
    .withTimeout(10.)
    .until(joystickInterrupt));

    // low
    Trigger aButton = new JoystickButton(m_armController, XboxController.Button.kA.value);
    aButton.onTrue(new MoveArmSequence(302.6662, .25, m_arm)
    .withTimeout(10.)
    .until(joystickInterrupt));

    // Ground Intake
    Trigger xButton = new JoystickButton(m_armController, XboxController.Button.kX.value);
    xButton.onTrue(new MoveArmSequence(315., 0.5, m_arm)
    .withTimeout(10.)
    .until(joystickInterrupt));

    // store
    Trigger rTrigger = new Trigger(() -> {
      return m_armController.getRightTriggerAxis() > .5;
    });
    rTrigger.onTrue(new MoveArmSequence(38., 0.35, m_arm)
      .withTimeout(10.)
      .until(joystickInterrupt));

    // Substation: HUMAN (player) (IS THAT AN UNDERTALE REFE-)
    Trigger lTrigger = new Trigger(() -> {
      return m_armController.getLeftTriggerAxis() > .5;
    });
    lTrigger.onTrue(new MoveArmSequence(100., 3.141592653589792, m_arm)
      .withTimeout(10.)
      .until(joystickInterrupt));
    /*
    Trigger bButton = new JoystickButton(m_xboxController, XboxController.Button.kB.value);
    bButton.onTrue(new MoveArm(m_arm, MoveArm.State.GROUND_ARM));
*/
    Trigger leftBumper2 = new JoystickButton(m_armController, XboxController.Button.kLeftBumper.value);
    leftBumper2.onTrue(new ClawCommand(pneumatics, true));

    Trigger rightBumper2 = new JoystickButton(m_armController, XboxController.Button.kRightBumper.value);
    rightBumper2.onTrue(new ClawCommand(pneumatics, false));
    // XboxController2 IS MOVEMENT NOT ARM
    // Trigger rightBumper = new JoystickButton(m_drivetrainController, XboxController.Button.kRightBumper.value);
    // rightBumper.whileTrue(new TurboModeCommand(m_drivetrain));

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

    //someone use case and switch because i mean come on
    if (m_chooser.getSelected().equals(m_scoreAutoString))
    {
      return scoreAuto;
    } else if (m_chooser.getSelected().equals(m_autoBalanceString)) {
      return balanceAuto;
    } else if (m_chooser.getSelected().equals(m_balanceNoMobilityString)) {
      return balanceNoMobilityAuto;
    }
    return yeetAuto;
  }
}