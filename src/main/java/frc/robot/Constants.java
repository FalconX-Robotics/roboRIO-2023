// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;
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
public final class Constants {
  
  // Drivetrain Ports
  public static final int FRONT_LEFT_MOTOR_PORT = 7;
  public static final int BACK_LEFT_MOTOR_PORT = 9;
  public static final int FRONT_RIGHT_MOTOR_PORT = 3;
  public static final int BACK_RIGHT_MOTOR_PORT = 5;

  // Arm Ports
  public static final int ARM_ROTATION_MOTOR_PORT = 8;
  public static final int ARM_EXTENSION_MOTOR_PORT = 6;

  public static final boolean LeftEncoderReversed = true;
  public static final boolean RightEncoderReversed = false;

  public static final int XBOX_CONTROLLER_PORT = 0;
  public static final int XBOX_CONTROLLER_PORT2 = 1;
  
  public static final int PIGEON_PORT = 42;
  public static final int LED_PORT = 10;

  // Gear Ratios
  public static final double ARM_ROTATION_GEAR_RATIO = 5.0 / 1.0 * 52. / 18. * 58. / 18. * 64. / 15.;
  public static final double DRIVETRAIN_GEARBOX = 50.0/12.0*50./24.;
  public static final double DRIVETRAIN_WHEEL_DIAMETER = 0.0254 * 6; // meters

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
