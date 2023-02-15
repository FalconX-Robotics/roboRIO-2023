// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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
    public static final int kDriverControllerPort = 1;
    public static final int kArmControllerPort = 0;
  }
  // Drivetrain Ports
  public static final int FRONT_LEFT_MOTOR_PORT = 2;
  public static final int BACK_LEFT_MOTOR_PORT = 7;
  public static final int FRONT_RIGHT_MOTOR_PORT = 1;
  public static final int BACK_RIGHT_MOTOR_PORT = 4;

  // Arm Ports
  public static final int ARM_ROTATION_MOTOR_PORT = 3;
  public static final int ARM_EXTENSION_MOTOR_PORT = 12;

  // WHAT IS AN ENCODER??? WHY WHAT HOW WHEN AAAAAA
  // temporary values
  // public static final int LeftEncoderPort1 = 6;
  // public static final int LeftEncoderPort2 = 6;
  // public static final int RightEncoderPort1 = 6;
  // public static final int RightEncoderPort2 = 2;
  public static final boolean LeftEncoderReversed = true;
  public static final boolean RightEncoderReversed = false;

  public static final int XBOX_CONTROLLER_PORT = 0;
  public static final int XBOX_CONTROLLER_PORT2 = 0;
  
  public static final int PIGEON_PORT = 42;
  public static final int LED_PORT = 10;
  
  // SysId, DO NOT CHANGE UNLESS YOU KNOW WHAT YOU ARE DOING
  // have a nice day - will
  public static final double ksVolts = 0.13012;
  public static final double kvVoltSecondsPerMeter = 2.3138;
  public static final double kaVoltSecondsSquaredPerMeter = 0.36023;
  public static final double kPDriveVel = 3.0519;
  // Distance between wheels
  public static final double kTrackWidthMeters = 0.53;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
  public static final double kMaxSpeedMetersPerSecons = 2.;
  public static final double kMaxAccelerationMetersPerSecondSquared = 1.;
  // Idk what a ramsete controller is but its important -w
  public static final double kRamseteB = 2.;
  public static final double kRamseteZeta = 0.7;
}
