package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase{
    // Define motors
    // Left motors
    private final CANSparkMax m_leftFrontMotor  = new CANSparkMax(Constants.FRONT_LEFT_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax m_leftBackMotor   = new CANSparkMax(Constants.BACK_LEFT_MOTOR_PORT, MotorType.kBrushless);
    private final MotorControllerGroup m_leftMotorGroup = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
    // Right motors
    private final CANSparkMax m_rightFrontMotor = new CANSparkMax(Constants.FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax m_rightBackMotor  = new CANSparkMax(Constants.BACK_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    private final MotorControllerGroup m_rightMotorGroup = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);
    
    private final double GEARING = 50.0/12.0*50./24.;
    private final double WHEEL_CIRCUMFERENCE = 0.1524 * Math.PI;
    public static boolean slowModeOn;

    PIDController pidController = new PIDController(92.2, 0, 7.3);

    private final RelativeEncoder m_leftEncoder = m_leftFrontMotor.getEncoder();
    private final RelativeEncoder m_rightEncoder = m_rightFrontMotor.getEncoder();

    // Drivetrain & gyro
    private final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(Constants.PIGEON_PORT);
    private final DifferentialDrive m_drivetrain = new DifferentialDrive(m_leftMotorGroup, m_rightMotorGroup);

    // Odometry supposedly tracks the position over time?
    private DifferentialDriveOdometry m_odometry;
    
    private final SlewRateLimiter m_leftRateLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter m_rightRateLimiter = new SlewRateLimiter(1);
    
    public PigeonIMU getGyro() {
        return m_gyro;
    }
    
    public Drivetrain () {
        m_leftFrontMotor.setInverted(true);
        m_leftBackMotor.setInverted(true);
        m_rightFrontMotor.setInverted(false);
        m_rightBackMotor.setInverted(false);

        m_leftFrontMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_rightFrontMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_leftBackMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_rightBackMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // do we even have encoders? -w yes we do -also william
        m_leftEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE/GEARING);
        m_leftEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE/GEARING/60);//magic number :D
        m_rightEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE/GEARING);
        m_rightEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE/GEARING/60);

        // m_leftEncoder.setMeasurementPeriod(0);
        resetLiterallyAlmostEverythingForAuto();
    }
        // m_leftEncoder.setPositionConversionFactor(0.4788);
        // m_leftEncoder.setVelocityConversionFactor(0.4788);
        // m_rightEncoder.setPositionConversionFactor(0.4788);
        // m_rightEncoder.setVelocityConversionFactor(0.4788);
        m_odometry = new DifferentialDriveOdometry(
            m_gyro.getRotation2d(),
            m_leftEncoder.getPosition(),
            m_rightEncoder.getPosition()
        );

        m_leftFrontMotor.setSmartCurrentLimit(60);
        m_leftBackMotor.setSmartCurrentLimit(60);
        m_rightFrontMotor.setSmartCurrentLimit(60);
        m_rightBackMotor.setSmartCurrentLimit(60);
       }
    // Command base -> ab
    // private Command command = new
    // private CommandBase command = new ArcadeDrive();
    public void periodic () {
        SmartDashboard.putNumber("Left Motor Group", m_leftMotorGroup.get());
        SmartDashboard.putNumber("Right Motor Group", m_rightMotorGroup.get());
        m_odometry.update(m_gyro.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());
        SmartDashboard.putNumber("leftEncoderPositionMeters", getLeftPositionMeters());
        SmartDashboard.putNumber("rightEncoderPositionMeters", getRightPositionMeters());
        SmartDashboard.putNumber("poseX", getPose().getX());
        SmartDashboard.putNumber("poseY", getPose().getY());
        SmartDashboard.putNumber("Rot", m_gyro.getYaw());
        SmartDashboard.putString("pose2d", m_odometry.toString());
        SmartDashboard.putNumber("SpeedMetersPerSecond", m_rightEncoder.getVelocity());

    }
    public double getLeftPositionMeters () {
        return -m_leftEncoder.getPosition();
    }
    public double getRightPositionMeters () {
        return -m_rightEncoder.getPosition();
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
        return new DifferentialDriveWheelSpeeds(-(m_leftEncoder.getVelocity()), -(m_rightEncoder.getVelocity()));
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
    public RelativeEncoder getLeftEncoder () {
        return m_leftEncoder;
    }
    public RelativeEncoder getRightEncoder () {
        return m_rightEncoder;
    }
    /** Also prob unneccessary */
    public void zeroHeading(){
        m_gyro.reset();
    }
    public double getTurnRate() {
        return -m_gyro.getRate();
    }

    public void resetLiterallyAlmostEverythingForAuto() {
        resetEncoders();
        m_gyro.reset();
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
        m_leftMotorGroup.set(m_leftRateLimiter.calculate((slowModeOn ? leftPercentOutput / 3 : leftPercentOutput)));
        m_rightMotorGroup.set(m_rightRateLimiter.calculate((slowModeOn ? rightPercentOutput / 3: rightPercentOutput)));
        // System.out.println("setting motors " + leftPercentOutput + ", " + rightPercentOutput);
        m_drivetrain.feed();
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
        leftVoltage = MathUtil.clamp(leftVoltage, -3, 3);
        rightVoltage = MathUtil.clamp(rightVoltage, -3, 3);
        m_leftMotorGroup.setVoltage(leftVoltage);
        m_rightMotorGroup.setVoltage(rightVoltage);
        
        //System.out.println("VoltSet: " + leftVoltage + ", " + rightVoltage);
        m_drivetrain.feed(); // idk what this does but it sounds cool
    }
    // Define arcadeDrive
        // We dont ascribe left or right in case we want to map both to one joystick
    public void arcadeDrive (double fowardPercentOutput, double turnPercent) {
        if (slowModeOn) {
            m_drivetrain.arcadeDrive(fowardPercentOutput / 3, turnPercent / 2);
        } else { 
            m_drivetrain.arcadeDrive(fowardPercentOutput, turnPercent);
        }
    }

    //polymomomomomomoprhisim
            //   help it hhurts
            // :(
                // aiya

    public void curvatureDrive (double leftPercentY, double rightPercentY, boolean turnInPlace) {
        m_drivetrain.curvatureDrive(
            m_leftRateLimiter.calculate(leftPercentY  * (slowModeOn ? 0.33 : 1)),
            m_rightRateLimiter.calculate(rightPercentY * ((slowModeOn && turnInPlace) ? 0.33 : 1)), 
            turnInPlace);
    }

    public void pidTankDrive(double distance) {
        double setpoint = distance;
        pidController.setSetpoint(setpoint);

        m_leftMotorGroup.set(pidController.calculate(1));
        m_rightMotorGroup.set(pidController.calculate(1));
    }

    
    private double getMeasurement() {
        // TODO Auto-generated method stub
        return 0;
    }

    
    private void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        
    }

    private void ResetEncoders() {
        m_leftBackMotor.getEncoder().setPosition(0);
        m_leftFrontMotor.getEncoder().setPosition(0);
        m_rightBackMotor.getEncoder().setPosition(0);
        m_rightFrontMotor.getEncoder().setPosition(0);
    }
}


/*
 * No ones ever going to see this so im putting this down here
 * PID
 *  P - 92.2
 *  I- P / D - 12.6
 *  D - 7.3
 */