// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * <div> Nobody will know, because at the end of the day it is night.
 * <p>The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * <h1> William was here
 * <h2> Logan was too 
 */
import edu.wpi.first.wpilibj.XboxController;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;


  }
  // Drivetrain Ports
  public static final int FRONT_LEFT_MOTOR_PORT = 2;
  public static final int BACK_LEFT_MOTOR_PORT = 7;
  public static final int FRONT_RIGHT_MOTOR_PORT = 1;
  public static final int BACK_RIGHT_MOTOR_PORT = 4;

  public static final int XBOX_CONTROLLER_PORT = 0;
  
  public static final int LED_PORT = 10;
}
