// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // *****************************************
  // ********** DRIVE TRAIN ******************
  // *****************************************
  public static final class DriveConstants {
    public static final int RIGHT_MOTOR_1 = 3;
    public static final int RIGHT_MOTOR_2 = 2;
    public static final int RIGHT_MOTOR_3 = 14;
    public static final int LEFT_MOTOR_1 = 4;
    public static final int LEFT_MOTOR_2 = 5;
    public static final int LEFT_MOTOR_3 = 6;

    public static final int SHIFTER_FORWARD_CHANNEL = 8;
    public static final int SHIFTER_REVERSE_CHANNEL = 9;

    // Drivetrain Speed
    public static final double DRIVETRAINSPEED = 1;

    // Duration of DRIVEFORWARD Command
    public static final double DRIVE_FORWARD_TIME = 3.0;

    // Speed During AUTO
    public static final double AUTONOMOUS_SPEED = 0.4;

    /////////////// SYSID VALUES ///////////////

    // TODO: Find real values for the love of god
    public static final double ksVolts = 0.58;
    public static final double kvVoltSecondsPerMeter = 3.44;
    public static final double kaVoltSecondsSquaredPerMeter = 0.165;

    public static final double kPDriveVel = 6.3;

    public static final double kTrackwidthMeters = 0.74551;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  }

  // *****************************************
  // ************** SHOOTER ******************
  // *****************************************
  public static final class Shooters {
    public static final int SHOOTER_MOTOR_1 = 8;
    public static final int SHOOTER_MOTOR_2 = 7;
    public static final int TURRET_MOTOR = 9;

    public static final double AUTO_SHOOT_TIME = 6.0;
    public static final double LL_SHOT_HEIGHT = 5;
    public static final double SHOOT_SPEED = 0.68;
    // good speed value is 0.38
    public static final double BACKSPIN_SPEED = 0.48;
    // good speed value is 0.45

    ////////// LIMELIGHT VALUES////////////

    // TODO: Get real angles for Lensheight and MountAngle
    public static final double MountAngleDegrees = 45;
    public static final double LensHeightInches = 30;
    public static final double GoalHeightInches = 104;
  }

  // *****************************************
  // ************** INTAKE *******************
  // *****************************************
  public static final class Ctake {
    public static final int INTAKE_MOTOR_1 = 15;
    public static final double INTAKE_SPEED = 0.75;

    // Pnuematics
    public static final int MODULE_NUMBER = 20;
    public static final int INTAKE_PISTON_FORWARD_CHANNEL = 6;
    public static final int INTAKE_PISTON_REVERSE_CHANNEL = 7;
  }

  // *****************************************
  // ************** JOYSTICKS ****************
  // *****************************************

  public static final int LEFT_JOY = 0;
  public static final int RIGHT_JOY = 1;
  public static final int XBOXCONTROLLER = 2;
  public static final int XBOXCONTROLLER2 = 3;

  // ***************************************
  // ************** STORAGE ****************
  // ***************************************

  public static final int INDEX_MOTOR = 12;
  public static final double INDEX_SPEED = 1;

  // *****************************************
  // *************** Settings ****************
  // *****************************************

  public static final Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();

  public static final class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = 3.048;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.328;

    // TODO: Find deez
    public static final double INTAKE_EXTEND = 1.0;
    public static final double SHOOTER_INITIALIZE = 1.5;
    public static final double INDEX_BALL = 2.5;
    public static final double HUMAN_WAIT = 2.0;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
