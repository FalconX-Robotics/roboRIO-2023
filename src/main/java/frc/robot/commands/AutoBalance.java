// package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.subsystems.Drivetrain;

// public class AutoBalance extends PIDCommand {
//     private Drivetrain m_drivetrain;
//     protected double m_P, m_I, m_D, m_F;
//     protected double m_positionTolerance = 0, m_velocityTolerance = 0;
//     protected double m_maxSpeed = 1;

//     protected DoubleSupplier m_setpointSupplier;
// 	protected NetworkTableEntry m_errorField = SmartDashboard.getEntry("DriveForward/Error");
//     protected NetworkTableEntry m_velocityField = SmartDashboard.getEntry("DriveForward/Velocity");
    
//     /**
//      * @param setpointSource is only called on initialize
//      */
//     public AutoBalance(DoubleSupplier setpointSource, Drivetrain drivetrain) {
//         super(new PIDController(0, 0, 0),
//             () -> 0.,
//             0.,
//             output -> {},
//             drivgitetrain);
//         m_drivetrain = drivetrain;
//         m_controller.setPID(m_P, m_I, m_D);
//         m_controller.setTolerance(m_positionTolerance, m_velocityTolerance);
//     }

//     public AutoBalance(double setpointSource, Drivetrain drivetrain) {
//         this(() -> setpointSource, drivetrain);
//     }

//     public void setPIDF(double P, double I, double D, double F) {
//         m_P = P;
//         m_I = I;
//         m_D = D;
//         m_F = F;
//         m_controller.setPID(P, I, D);
//     }

//     public void setSetpoint(double setpoint) {
//         this.m_setpoint = () -> setpoint;
//     }

//     @Override
//     public void initialize() {
//         double setpoint = m_setpointSupplier.getAsDouble();
//         m_setpoint = () -> setpoint;
//         m_useOutput = output -> m_drivetrain.arcadeDrive(m_F*Math.signum(output) + MathUtil.clamp(output, -m_maxSpeed, m_maxSpeed), 0);

//         // double m_initEncoderDistance = m_drivetrain.averageEncoderDistance();
//         // m_measurement = () -> m_drivetrain.averageEncoderDistance() - m_initEncoderDistance;
//     }

//     @Override
//     public void execute() {
//         super.execute();
//         m_errorField.setDouble(m_controller.getPositionError());
//         m_velocityField.setDouble(m_controller.getVelocityError());
//     }

//     @Override
//     public boolean isFinished() {
//         return getController().atSetpoint();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         m_errorField.setDouble(100);
//     }
// }
// import java.util.function.DoubleConsumer;
// import java.util.function.DoubleSupplier;

// import com.ctre.phoenix.sensors.PigeonIMU;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.subsystems.Drivetrain;

// public class AutoBalance extends PIDCommand {
//     // public AutoBalance (Drivetrain drivetrain) {}
//   private final PIDController m_controller;
//   private DoubleSupplier m_measurement;
//   private DoubleSupplier m_setpoint;
//   private DoubleConsumer m_useOutput;

//     Drivetrain m_drivetrain;
//     PigeonIMU m_gyro;

//     public AutoBalance(Drivetrain drivetrain) {
//         super(m_controller, m_setpoint, m_measurement, m_useOutput, null)
//         // Will start auto for 3 seconds and then turn off
//         m_drivetrain = drivetrain;
//         m_gyro = drivetrain.getGyro();
//         addRequirements(m_drivetrain);
//     }

// }



// note for later this is because i want to run the robot and this has errors i dont want to deal with rn
