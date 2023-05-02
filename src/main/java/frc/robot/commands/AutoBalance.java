package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase  {

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
    boolean updateDistance = true;
    double forward = 1;
    PIDController pidController = new PIDController(0.006, 0, 0.0005);

    public AutoBalance(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    public AutoBalance(Drivetrain drivetrain, double direction) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
        forward = direction;
    }
    
    @Override
    public void initialize() {
        m_drivetrain.printInverted();
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
            m_drivetrain.voltTankDrive(-0.25 * forward * 12, -0.25 * forward *12);
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
        if (updateDistance) {
            distanceToCenter = (-50 * forward) * 0.0254 - distanceMoved;
        }
        
        if (Math.abs(distanceToCenter) < 0.0254 ) {
            distanceToCenter = 0;
            updateDistance = false;
        }

        // double driveSpeed = distanceToCenter * 0.2 + (MathUtil.clamp(gyro.getPitch(), -15, 15)) * -0.01;
        double driveSpeed = distanceToCenter * 0.2;
        driveSpeed += pidController.calculate((MathUtil.clamp(gyro.getPitch(), -15, 15)), 0);
        driveSpeed = MathUtil.clamp(driveSpeed, -0.3, 0.3);
       

        m_drivetrain.voltTankDrive(driveSpeed * 12, driveSpeed * 12);
        
        

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

    public static int multiplyPositiveInts(int x, int y) {
        int yPower = (int) (Math.log(y)/Math.log(2));
        int yRemainder = y % yPower;
        int result = x;

        result = result << yPower;
        for (int i = 0; i < yRemainder; i++) {
          result += x;
        }
        return result;

        /*
         * 3, 17
         * 00000011 - 3
         * 00010001 - 17 = 2^5 rem 1
         * 
         * 
         * 
         * 
         * 
        */
    }
    
    
}