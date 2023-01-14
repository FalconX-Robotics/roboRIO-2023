package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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
    
    // Drivetrain
    private final DifferentialDrive m_drivetrain = new DifferentialDrive(m_leftMotorGroup, m_rightMotorGroup);

    // Define tankDrive
        // Both using y
    public void tankDrive (double leftPercentOutput, double rightPercentOutput) {
        m_leftMotorGroup.set(leftPercentOutput);
        m_rightMotorGroup.set(rightPercentOutput);
    }
    // Define arcadeDrive
        // We dont ascribe left or right in case we want to map both to one joystick
    public void arcadeDrive (double PercentOutputY, double PercentOutputX) {
        m_drivetrain.arcadeDrive(PercentOutputY, PercentOutputX);
    }

    
}
