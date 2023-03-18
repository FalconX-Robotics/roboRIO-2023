package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
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
        addRequirements(m_drivetrain);
        gyro = m_drivetrain.getGyro();
        gyro.setYaw(0.0);
        startingPitch = gyro.getPitch();
    }

    @Override
    public void execute() {
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
            m_balanceState = State.MOVE_UP;
        }
    }

    private void moveUp() {
        if (Math.abs(gyro.getPitch() - startingPitch) > 6) {
            m_drivetrain.tankDrive(-0.2, -0.2);
        } else {
            m_drivetrain.resetEncoders();
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
        double driveSpeed = distanceToCenter * 0.01 + gyro.getYaw() * 0.01;
        
        m_drivetrain.tankDrive(driveSpeed, driveSpeed);
        
        distanceToCenter = 23 - distanceMoved;
        // robot is 28 inches long
        // robot should move 23 inches

        // if (distanceToCenter <= 0.5) {
        //     balanced = true;
        // }
        counter++;
        if (counter > 500) {
            balanced = true;
        }
    }
}
