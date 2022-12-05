// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.rambots4571.rampage.util.LinearInterpolator;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  // ************* DRIVE TRAIN ***************
  // *****************************************
  public static final class DriveConstants {
    public static final int RIGHT_MASTER = 3;
    public static final int RIGHT_MOTOR_2 = 2;
    public static final int RIGHT_MOTOR_3 = 14;
    public static final int LEFT_MASTER = 4;
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

    public static Boolean USING_GYRO = true;

    /////////////// SYSID VALUES ///////////////

    public static final double ksVolts = 0.558;
    public static final double kvVoltSecondsPerMeter = 3.4335;
    public static final double kaVoltSecondsSquaredPerMeter = 0.171;

    public static final double kPDriveVel = 3.2137;

    public static final int kCountsPerRev = 2048;

    //TODO: WTF IS THIS MELODY
    public static final int k100msPerSec = 10;

    // Low gear for drivetrain
    public static final double kGearRatio = 15.32;

    public static final double kWheelRadiusInches = 3.0;

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
    public static final int SHOOTER_MOTOR = 7;
    public static final int BACKSPIN_MOTOR = 8;

    public static final double AUTO_SHOOT_TIME = 6.0;

    /////////// SHOOTER SPEEDS //////////////

    public static final double SHOOT_SPEED = 0.50;
    public static final double BACKSPIN_SPEED = 0.25;

    // TODO: UPADTE THESE SPEEDS FROM THE IRON RAM LAPTOP

    public static final double INSIDE_SPEED = .50;
    public static final double INSIDE_BSPEED = 0.40;

    public static final double TARMAC_SPEED = 0.50;
    public static final double TARMAC_BSPEED = 0.42;

    public static final double CAGE_SPEED = 0.45;
    public static final double CAGE_BSPEED = 0.42;

    public static final double LAUNCHPAD_SPEED = 0.55;
    public static final double LAUNCHPAD_BSPEED = 0.42;

    ////////// LIMELIGHT VALUES ////////////

    public static final double MountAngleDegrees = 28;
    public static final double LensHeightInches = 31.5;
    public static final double GoalHeightInches = 104;

    ////////// DISTANCE//SPEED DATA /////////

    // TODO: change the voltage to raw units
    public static final double[][] SHOOTER_SPEED_ARRAY = {
      {89, 2944},
      {136.5, 2985},
      {176.5, 3855}
    };

    public static final double[][] BACKSPIN_SPEED_ARRAY = {
      {89, 2343},
      {136.5, 2378},
      {176.5, 3199}
    };

    public static final LinearInterpolator SHOOTER_SPEED_INTERPOLATOR =
        new LinearInterpolator(SHOOTER_SPEED_ARRAY);
    public static final LinearInterpolator BACKSPIN_SPEED_INTERPOLATOR =
        new LinearInterpolator(BACKSPIN_SPEED_ARRAY);

    // TODO: Find real error
    public static final double MAX_SPEED_ERROR = 0.03;
  }

  public static class SFF {

    public static final double Ks = 0.53467;
    public static final double Kv = 0.10845;
    public static final double Ka = 0.01518;

    public static SimpleMotorFeedforward getShooterFF() {
      return new SimpleMotorFeedforward(SFF.Ks, SFF.Kv, SFF.Ka);
    }
  }

  public static class BFF {

    public static final double Ks = 0.48369;
    public static final double Kv = 0.10595;
    public static final double Ka = 0.0023568;

    public static SimpleMotorFeedforward getBackSpinFF() {
      return new SimpleMotorFeedforward(BFF.Ks, BFF.Kv, BFF.Ka);
    }
  }

  // *****************************************
  // ************** INTAKE *******************
  // *****************************************
  public static final class Ctake {
    public static final int INTAKE_MOTOR = 15;
    public static final double INTAKE_SPEED = 0.5;

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

  public static final int timeoutMs = 10;

  public static final Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();

  public static final class Odometry {
    public static Translation2d STARTING_TRANSLATION = new Translation2d();
    public static Rotation2d STARTING_ANGLE = new Rotation2d();
    public static Pose2d STARTING_POSITION = new Pose2d(STARTING_TRANSLATION, STARTING_ANGLE);
  }

  public static final class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = 3.048;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.328;

    // TODO: Find deez
    public static final double INTAKE_EXTEND = 1.0;
    public static final double SHOOTER_INITIALIZE = 1.5;
    public static final double INDEX_BALL = 2.5;
    public static final double HUMAN_WAIT = 2.0;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
