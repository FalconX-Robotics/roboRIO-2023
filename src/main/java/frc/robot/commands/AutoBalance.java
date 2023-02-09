package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {

    public static enum BalanceState {
        MOVE_FORWARD,
        MOVE_UP,
        BALANCE
    }

    public BalanceState m_balanceState = BalanceState.MOVE_FORWARD;

    Drivetrain m_drivetrain;
    PigeonIMU gyro;
    boolean balanced = false;
    double startingPitch;

    

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
        if (balanced) {
            return balanced;
        }
        return false;
    }

    private void moveForward() {
        System.out.println("Pitch at " + gyro.getPitch());
        if (Math.abs(gyro.getPitch() - startingPitch) < 5) {
            m_drivetrain.tankDrive(-0.5, -0.5);
        } else {
            m_balanceState = State.MOVE_UP;
        }
    }

    private void moveUp() {
        if (Math.abs(gyro.getPitch() - startingPitch) > 5) {
            m_drivetrain.tankDrive(-0.3, -0.3);
        } else {
            m_balanceState = State.BALANCE;
        }
    }

    private void balance() {
        balanced = true;
    }
}
