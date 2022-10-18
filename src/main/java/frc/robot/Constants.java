// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.rambots4571.rampage.util.LinearInterpolator;

import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

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
  // ********** DRIVE TRAIN ******************
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

    public static final double SHOOT_SPEED = 0.52;
    public static final double BACKSPIN_SPEED = 0.42;

    ////////// LIMELIGHT VALUES ////////////

    public static final double MountAngleDegrees = 28;
    public static final double LensHeightInches = 31.5;
    public static final double GoalHeightInches = 104;

    ////////// DISTANCE//SPEED DATA /////////

    public static final double[][] SHOOTER_SPEED_ARRAY = {
      {89, 0.5},
      {136.5, 0.52},
      {176.5, 0.65}
    };

    public static final double[][] BACKSPIN_SPEED_ARRAY = {
      {89, 0.4},
      {136.5, 0.42},
      {176.5, 0.55}
    };

    public static final LinearInterpolator SHOOTER_SPEED_INTERPOLATOR =
        new LinearInterpolator(SHOOTER_SPEED_ARRAY);
    public static final LinearInterpolator BACKSPIN_SPEED_INTERPOLATOR =
        new LinearInterpolator(BACKSPIN_SPEED_ARRAY);

    // TODO: Find real error
    public static final double MAX_SPEED_ERROR = 150;

    public static final double MIN_RPM = 100;

    public interface ShooterFF {
      double Ks = 0.53467;
      double Kv = 0.10845;
      double Ka = 0.01518;

      static SimpleMotorFeedforward getController() {
        return new SimpleMotorFeedforward(ShooterFF.Ks, ShooterFF.Kv, ShooterFF.Ka);
      }
    }

    public interface ShooterPID {
      double kP = 0.14636;
      double kI = 0.0;
      double kD = 0.0;

      static PIDController getController() {
        return new PIDController(kP, kI, kD);
      }
    }
    // TODO: Find how much time it takes for the shooter to get to 63.2% of the target speed
    public static final SmartNumber RC = new SmartNumber("Shooter RC", 0.3);
  }

  // *****************************************
  // ************** INTAKE *******************
  // *****************************************
  public static final class Ctake {
    public static final int INTAKE_MOTOR = 15;
    public static final double INTAKE_SPEED = 0.6;

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

  public static class Odometry {
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
