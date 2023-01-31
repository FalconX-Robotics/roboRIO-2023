package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {

    enum State {
        MOVE_FORWARD,
        MOVE_UP,
        BALANCE
    }

    State state = State.MOVE_FORWARD;

    Drivetrain m_drivetrain;
    PigeonIMU gyro;
    boolean balanced = false;

    

    AutoBalance(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }
    
    @Override
    public void initialize() {
        addRequirements(m_drivetrain);
        gyro = m_drivetrain.getGyro();
        gyro.setYaw(0.0);
    }

    @Override
    public void execute() {
        switch (state) {
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
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        if (balanced) {
            return balanced;
        }
        return false;
    }

    private void moveForward() {
        if (!(gyro.getPitch() > 15)) {
            m_drivetrain.tankDrive(0.3, 0.3);
        } else {
            state = State.MOVE_UP;
        }

    }

    private void moveUp() {
        if (gyro.getPitch() > 5) {
            m_drivetrain.tankDrive(0.3, 0.3);
        } else {
            state = State.BALANCE;
        }
    }

    private void balance() {
        balanced = true;
    }
}
