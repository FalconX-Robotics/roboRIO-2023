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
            m_balanceState = State.BALANCE;
        }
    }

    //unused
    private void moveUp() {
        if (Math.abs(gyro.getPitch() - startingPitch) > 6) {
            m_drivetrain.tankDrive(-0.1, -0.1);
        } else {
            m_balanceState = State.BALANCE;
        }
    }

    private void balance() {
        double driveSpeed = distanceToCenter * 0.3 + gyro.getPitch() * -0.005;
        driveSpeed = MathUtil.clamp(driveSpeed, -0.3, 0.3);
       

        m_drivetrain.tankDrive(driveSpeed, driveSpeed);
        
        distanceToCenter = -23 * 0.0254 - distanceMoved;

        SmartDashboard.putNumber("Distance to center", distanceToCenter);
        SmartDashboard.putNumber("Distance moved", distanceMoved);
        SmartDashboard.putNumber("Drive speed", driveSpeed);
        // robot is 28 inches long
        // robot should move 23 inches

        // if (distanceToCenter <= 0.5) {
        //     balanced = true;
        // }


    }

    public static int add (int x, int y) {
        int[] array = new int[y];
        if (y > 1) {
            for (int i : array) {
                x = -~x;
            }
        } else if (y < 0) {
            for (int i = 0; i > y; i--) {
                x = ~-x;
            }
        }
        return x;
    }
}