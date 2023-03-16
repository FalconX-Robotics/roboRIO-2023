package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.simulation.MotorControllerSim;
import frc.robot.simulation.RelativeEncoderSim;

public class Drivetrain extends SubsystemBase{
    // Define motors
    // Left motors
    private final CANSparkMax m_leftFrontMotor  = new CANSparkMax(Constants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax m_leftBackMotor   = new CANSparkMax(Constants.BACK_LEFT_MOTOR_PORT, MotorType.kBrushless);
    private MotorController m_leftMotorGroup = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
    // Right motors
    private final CANSparkMax m_rightFrontMotor = new CANSparkMax(Constants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax m_rightBackMotor  = new CANSparkMax(Constants.BACK_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    private MotorController m_rightMotorGroup = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);
    
    private final double GEARING = 50.0/12.0*50./24.;
    private final double WHEEL_CIRCUMFERENCE = 0.1524 * Math.PI;
    // What is encoder
    // Deleted for now
    
    // private final Encoder m_leftEncoder = new Encoder(
    //     Constants.LeftEncoderPort1,
    //     Constants.LeftEncoderPort2,
    //     Constants.LeftEncoderReversed
    // );
    // private final Encoder m_rightEncoder = new Encoder(
    //     Constants.RightEncoderPort1,
    //     Constants.RightEncoderPort2,
    //     Constants.LeftEncoderReversed
    // );

    private RelativeEncoder m_leftEncoder = m_leftFrontMotor.getEncoder();
    private RelativeEncoder m_rightEncoder = m_rightFrontMotor.getEncoder();

    // Drivetrain & gyro
    private final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(Constants.PIGEON_PORT);
    private DifferentialDrive m_drivetrain = new DifferentialDrive(m_leftMotorGroup, m_rightMotorGroup);

    // Odometry supposedly tracks the position over time?
    private DifferentialDriveOdometry m_odometry;
    /** Logging for all our data to fix ododododo auto */
    DataLog m_log = DataLogManager.getLog();
    DoubleLogEntry m_leftPositionMetersLog = new DoubleLogEntry(m_log, "/drivetrain/leftPostionMeters");
    DoubleLogEntry m_rightPositionMetersLog = new DoubleLogEntry(m_log, "/drivetrain/rightPostionMeters");
    DoubleLogEntry m_rotLog = new DoubleLogEntry(m_log, "/drivetrain/rot");
    DoubleLogEntry m_leftVelocityLog = new DoubleLogEntry(m_log, "/drivetrain/leftVelocityMetersPerSecond");
    DoubleLogEntry m_rightVelocityLog = new DoubleLogEntry(m_log, "/drivetrain/rightVelocityMetersPerSecond");
    DoubleArrayLogEntry m_poseLog = new DoubleArrayLogEntry(m_log, "/drivetrain/pose");
    DoubleLogEntry m_rightMotorPercentOutputLog = new DoubleLogEntry(m_log, "/drivetrain/rightMotorPercentOutput");
    DoubleLogEntry m_leftMotorPercentOutputLog = new DoubleLogEntry(m_log, "/drivetrain/leftMotorPercentOutput");
    
    public Drivetrain () {
        m_leftFrontMotor.setInverted(true);
        m_leftBackMotor.setInverted(true);
        m_rightFrontMotor.setInverted(false);
        m_rightBackMotor.setInverted(false);

        // do we even have encoders?
        // m_leftEncoder.setPositionConversionFactor(0.4788);
        // m_leftEncoder.setVelocityConversionFactor(0.4788);
        // m_rightEncoder.setPositionConversionFactor(0.4788);
        // m_rightEncoder.setVelocityConversionFactor(0.4788);
        // m_leftEncoder.setInverted(true);
        // m_rightEncoder.setInverted(true);
        m_leftEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE/GEARING);
        m_leftEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE/GEARING/60);//magic number :D
        m_rightEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE/GEARING);
        m_rightEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE/GEARING/60);

        // m_leftEncoder.setMeasurementPeriod(0);
        resetLiterallyAlmostEverythingForAuto();

        if (RobotBase.isSimulation()) {
            m_drivetrainSimulator = new DifferentialDrivetrainSim(
                DCMotor.getNEO(2), 
                8.89,
                4,
                30,
                Units.inchesToMeters(3), 
                0.7, 
                VecBuilder.fill(0.001, 0.001, 0.001, 0.01, 0.01, 0.005, 0.005));

            m_leftEncoder = new RelativeEncoderSim();
            m_rightEncoder = new RelativeEncoderSim();
            m_leftMotorGroup = new MotorControllerSim("Left Motor Group");
            m_rightMotorGroup = new MotorControllerSim("Right Motor Group");
            m_drivetrain = new DifferentialDrive(m_leftMotorGroup, m_rightMotorGroup);
        }
    }
    // Command base -> ab
    // private Command command = new
    // private CommandBase command = new ArcadeDrive();
    public void periodic () {
        SmartDashboard.putNumber("Left Motor Group", m_leftMotorGroup.get());
        SmartDashboard.putNumber("Right Motor Group", m_rightMotorGroup.get());
        m_rightMotorPercentOutputLog.append(m_rightMotorGroup.get());
        m_leftMotorPercentOutputLog.append(m_leftMotorGroup.get());
        m_odometry.update(m_gyro.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());
        SmartDashboard.putNumber("leftEncoderPositionMeters", getLeftPositionMeters());
        m_leftPositionMetersLog.append(getLeftPositionMeters());
        SmartDashboard.putNumber("rightEncoderPositionMeters", getRightPositionMeters());
        m_rightPositionMetersLog.append(getRightPositionMeters());
        SmartDashboard.putNumber("poseX", getPose().getX());
        SmartDashboard.putNumber("poseY", getPose().getY());
        SmartDashboard.putNumber("Rot", m_gyro.getYaw());
        m_rotLog.append(m_gyro.getYaw());
        SmartDashboard.putString("pose2d", m_odometry.toString());
        SmartDashboard.putNumber("SpeedMetersPerSecond", m_rightEncoder.getVelocity());
        m_leftVelocityLog.append(getWheelSpeeds().leftMetersPerSecond);
        m_rightVelocityLog.append(getWheelSpeeds().rightMetersPerSecond);
        Pose2d pose = getPose();
        double[] poseRaw = {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
        m_poseLog.append(poseRaw);
    }
    public double getLeftPositionMeters () {
        return m_leftEncoder.getPosition();
    }
    public double getRightPositionMeters () {
        return m_rightEncoder.getPosition();
    }
    // ODOMETRY CODE BELOW
    // DO NOT TOUCH UNLESS YOU ALREADY KNOW WHAT YOU ARE DOIN
    // wil :)
    public Pose2d getPose() {
        //System.out.println("Pose: " + m_odometry.getPoseMeters().getX() + ", "
          //  + m_odometry.getPoseMeters().getY() + ", "
            //+ m_odometry.getPoseMeters().getRotation().getDegrees());
        return m_odometry.getPoseMeters();
    }
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // supposed to be degrees/sec, not revolution/min
        // encoders are reversed :/
        return new DifferentialDriveWheelSpeeds((m_leftEncoder.getVelocity()), (m_rightEncoder.getVelocity()));
    }
    public void resetOdometry(){
        // this isnt what max said to do i dont think but i mean :/
        m_odometry.resetPosition(new Rotation2d(), 0., 0., new Pose2d());
    }
    public void setOdometry(Rotation2d rotation, double leftDistanceMeters, double rightDistanceMeters, Pose2d pose) {
        m_odometry.resetPosition(rotation, leftDistanceMeters, rightDistanceMeters, pose);
    }

    /** Prob unneccessary */
    public void resetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }
    private RelativeEncoder getLeftEncoder () {
        return m_leftEncoder;
    }
    private RelativeEncoder getRightEncoder () {
        return m_rightEncoder;
    }
    /** Also prob unneccessary */
    public void zeroHeading(){
        m_gyro.reset();
        m_gyro.setYaw(180);
    }
    public double getTurnRate() {
        return -m_gyro.getRate();
    }
    /**yes */
    public void resetLiterallyAlmostEverythingForAuto() {
        resetEncoders();
        zeroHeading();
        m_odometry = new DifferentialDriveOdometry(
            m_gyro.getRotation2d(),
            getLeftPositionMeters(),
            getRightPositionMeters()            
        );
    }
    // Define tankDrive
        // Both using y
    public void tankDrive (double leftPercentOutput, double rightPercentOutput) {
        m_leftMotorGroup.set(leftPercentOutput);
        m_rightMotorGroup.set(rightPercentOutput);
        // System.out.println("setting motors " + leftPercentOutput + ", " + rightPercentOutput);
    }
    /** Please do not use for actual driving
     *  as it would really suck at driving.
     *  Allows for auto code stuff,
     *  don't make me explain -w
     */
    public void voltTankDrive (double leftVoltage, double rightVoltage) {
        // m_leftFrontMotor.setInverted(false);
        // m_leftBackMotor.setInverted(false);
        // m_rightFrontMotor.setInverted(true);
        // m_rightBackMotor.setInverted(true);
        m_leftMotorGroup.setVoltage(leftVoltage);
        m_rightMotorGroup.setVoltage(rightVoltage);
        
        //System.out.println("VoltSet: " + leftVoltage + ", " + rightVoltage);
        m_drivetrain.feed(); // idk what this does but it sounds cool
    }
    // Define arcadeDrive
        // We dont ascribe left or right in case we want to map both to one joystick
    public void arcadeDrive (double fowardPercentOutput, double turnPercent) {
        m_drivetrain.arcadeDrive(fowardPercentOutput, turnPercent);
    }

    public void curvatureDrive (double leftPercentY, double rightPercentY) {
        m_drivetrain.curvatureDrive(leftPercentY, rightPercentY, false);
    }

    private DifferentialDrivetrainSim m_drivetrainSimulator;
    private BasePigeonSimCollection m_gyroSim = m_gyro.getSimCollection();
    private Field2d m_field = new Field2d();
    @Override
    public void simulationPeriodic() {
        m_drivetrainSimulator.setInputs(m_leftMotorGroup.get() * RobotController.getInputVoltage(),
                m_rightMotorGroup.get() * RobotController.getInputVoltage());

        m_drivetrainSimulator.update(0.02);

        RelativeEncoderSim leftEncoder = (RelativeEncoderSim) m_leftEncoder;
        RelativeEncoderSim rightEncoder = (RelativeEncoderSim) m_rightEncoder;

        // physical encoders are reversed from wheel direction
        leftEncoder.setSimulationPositionMeters(-m_drivetrainSimulator.getLeftPositionMeters());
        rightEncoder.setSimulationPositionMeters(-m_drivetrainSimulator.getRightPositionMeters());
        leftEncoder.setSimulationVelocityMetersPerSecond(-m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
        rightEncoder.setSimulationVelocityMetersPerSecond(-m_drivetrainSimulator.getRightVelocityMetersPerSecond());

        m_gyroSim.setRawHeading(m_drivetrainSimulator.getHeading().getDegrees());

        Pose2d pose = m_odometry.getPoseMeters();
        m_field.setRobotPose(new Pose2d(pose.getX() + 3, pose.getY() + 3, pose.getRotation()));
        SmartDashboard.putData(m_field);
    }
}
