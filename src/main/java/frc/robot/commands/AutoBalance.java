package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {

    public static enum State {
        MOVE_FORWARD,
        MOVE_UP,
        BALANCE
    }

    public State m_balanceState = State.MOVE_FORWARD;

    Drivetrain m_drivetrain;
    PigeonIMU gyro;
    boolean balanced = false;
    double startingPitch;
    int counter = 0;
    double distanceToCenter = 0;
    double distanceMoved;

    public AutoBalance(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        m_balanceState = State.MOVE_FORWARD;
        gyro = m_drivetrain.getGyro();
        gyro.setYaw(0.0);
        startingPitch = gyro.getPitch();
    }

    @Override
    public void execute() {
        SmartDashboard.putString("AutoBalance State", m_balanceState.toString());
        SmartDashboard.putNumber("Gyro pitch", gyro.getPitch());
        distanceMoved = m_drivetrain.getDistance();
        switch (m_balanceState) {
            case MOVE_FORWARD:
                moveForward();
                break;
            case MOVE_UP:
                moveUp();
                break;
            case BALANCE:
                
                balance();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return balanced;

    }

    private void moveForward() {
        System.out.println("Pitch at " + gyro.getPitch());
        if (Math.abs(gyro.getPitch() - startingPitch) < 7) {
            m_drivetrain.tankDrive(-0.25, -0.25);
        } else {
            m_drivetrain.resetEncoders();
            m_balanceState = State.MOVE_UP;
        }
    }

    private void moveUp() {
        if (Math.abs(gyro.getPitch() - startingPitch) > 6) {
            m_drivetrain.tankDrive(-0.2, -0.2);
        } else {
            m_balanceState = State.BALANCE;
        }
    }

    private void balance() {
        // if(counter < 50) {
            // if (gyro.getYaw() > 3) {
            //     System.out.println("going backwards, counter at "  + counter + " gyro at "+ gyro.getYaw());
            //     m_drivetrain.tankDrive(0.01 * gyro.getYaw(), 0.01 * gyro.getYaw());

            // } else if (gyro.getYaw() < -3) {
            //     System.out.println("going forwards, counter at "  + counter + " gyro at "+ gyro.getYaw());
            //     m_drivetrain.tankDrive(-0.01 * gyro.getYaw(), -0.01  * gyro.getYaw());

            // } else if (gyro.getYaw() > -3 && gyro.getYaw() < 3) {
            //     System.out.println("stopped, counter at "  + counter + " gyro at "+ gyro.getYaw());
            //     m_drivetrain.tankDrive(0, 0);

            // }
        // } else {
        //     m_drivetrain.tankDrive(0, 0);
        // }
        double driveSpeed = distanceToCenter * 0.0 + gyro.getPitch() * -0.005;
        driveSpeed = MathUtil.clamp(driveSpeed, -0.3, 0.3);
        if (Math.abs(gyro.getPitch()) < 1.5) {
            driveSpeed = 0;
        }

        m_drivetrain.tankDrive(driveSpeed, driveSpeed);
        
        distanceToCenter = 23 * 0.0254 - distanceMoved;

        SmartDashboard.putNumber("Distance to center", distanceToCenter);
        SmartDashboard.putNumber("Distance moved", distanceMoved);
        SmartDashboard.putNumber("Drive speed", driveSpeed);
        // robot is 28 inches long
        // robot should move 23 inches

        // if (distanceToCenter <= 0.5) {
        //     balanced = true;
        // }


    }
}
