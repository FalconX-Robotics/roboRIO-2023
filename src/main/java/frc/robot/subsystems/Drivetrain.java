package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
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

    PIDController pidController = new PIDController(92.2, 0, 7.3);
    // What is encoder
    // Depracated for now
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

    private final RelativeEncoder m_leftEncoder = m_leftFrontMotor.getEncoder();
    private final RelativeEncoder m_rightEncoder = m_rightFrontMotor.getEncoder();

    // Drivetrain & gyro
    private final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(Constants.PIGEON_PORT);
    private final DifferentialDrive m_drivetrain = new DifferentialDrive(m_leftMotorGroup, m_rightMotorGroup);

    // Odometry supposedly tracks the position over time?
    private final DifferentialDriveOdometry m_odometry;
    
    
    public PigeonIMU getGyro() {
        return m_gyro;
    }
    
    public Drivetrain () {
        m_leftFrontMotor.setInverted(true);
        m_leftBackMotor.setInverted(true);
        m_rightFrontMotor.setInverted(false);
        m_rightBackMotor.setInverted(false);

        // do we even have encoders?
        m_leftEncoder.setPositionConversionFactor(0.4788);
        m_leftEncoder.setVelocityConversionFactor(0.4788);
        m_rightEncoder.setPositionConversionFactor(0.4788);
        m_rightEncoder.setVelocityConversionFactor(0.4788);
        m_odometry = new DifferentialDriveOdometry(
            m_gyro.getRotation2d(),
            m_leftEncoder.getPosition(),
            m_rightEncoder.getPosition()            
        );
       }
    // Command base -> ab
    // private Command command = new
    // private CommandBase command = new ArcadeDrive();
    public void periodic () {
        SmartDashboard.putNumber("Left Motor Group", m_leftMotorGroup.get());
        SmartDashboard.putNumber("Right Motor Group", m_rightMotorGroup.get());
    }
    // Define tankDrive
        // Both using y
    public void tankDrive (double leftPercentOutput, double rightPercentOutput) {
        m_leftMotorGroup.set(leftPercentOutput);
        m_rightMotorGroup.set(rightPercentOutput);
        System.out.println("setting motors " + leftPercentOutput + ", " + rightPercentOutput);
    }
    // Define arcadeDrive
        // We dont ascribe left or right in case we want to map both to one joystick
    public void arcadeDrive (double fowardPercentOutput, double turnPercent) {
        m_drivetrain.arcadeDrive(fowardPercentOutput, turnPercent);
    }

    public void curvatureDrive (double leftPercentY, double rightPercentY) {
        m_drivetrain.curvatureDrive(leftPercentY, rightPercentY, false);
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
}


/*
 * No ones ever going to see this so im putting this down here
 * PID
 *  P - 92.2
 *  I- P / D - 12.6
 *  D - 7.3
 */