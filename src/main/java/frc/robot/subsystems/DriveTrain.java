// Copyright (c) Darius and other Darius contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file and DARI lisence in the root directory of this
// project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Ctake;
import frc.robot.Constants.DriveConstants;
import java.util.Arrays;
import java.util.List;

public class DriveTrain extends SubsystemBase {
  // Local Talon Variables
  private final WPI_TalonFX rightMaster;
  private final WPI_TalonFX leftMaster;

  private final WPI_TalonFX rightMotor2;
  private final WPI_TalonFX rightMotor3;
  private final WPI_TalonFX leftMotor2;
  private final WPI_TalonFX leftMotor3;

  private final List<WPI_TalonFX> allMotors;

  private final DoubleSolenoid shifter;

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry m_Odometry;

  private final AHRS navX;

  private final StatorCurrentLimitConfiguration statorLimitConfig =
    new StatorCurrentLimitConfiguration(true, 40, 70, 2);

  private final SupplyCurrentLimitConfiguration supplyLimitConfig =
    new SupplyCurrentLimitConfiguration(true, 40, 60, 4);

  private final NeutralMode neutralMode = NeutralMode.Brake;

  private double rampRate;
  private final int timeoutMs;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    // Talons
    rightMaster = new WPI_TalonFX(DriveConstants.RIGHT_MOTOR_1);
    rightMotor2 = new WPI_TalonFX(DriveConstants.RIGHT_MOTOR_2);
    rightMotor3 = new WPI_TalonFX(DriveConstants.RIGHT_MOTOR_3);
    leftMaster = new WPI_TalonFX(DriveConstants.LEFT_MOTOR_1);
    leftMotor2 = new WPI_TalonFX(DriveConstants.LEFT_MOTOR_2);
    leftMotor3 = new WPI_TalonFX(DriveConstants.LEFT_MOTOR_3);

    allMotors = Arrays.asList(rightMaster, rightMotor2, rightMotor3, leftMaster,
      leftMotor2, leftMotor3);

    // **********************************************
    // ************** Falcon Configs ****************
    // **********************************************

    // TODO: Adjust rampRate
    rampRate = 0.35;
    timeoutMs = 15;

    allMotors.forEach(motor -> {
      // Factory Resets all TalonFX
      motor.configFactoryDefault();
      // Sets Motor to Brake/Coast
      motor.setNeutralMode(neutralMode);
      // Current limit to prevent breaker tripping. Approx at 150% of rated
      // current supply.
      motor.configSupplyCurrentLimit(supplyLimitConfig);
      motor.configStatorCurrentLimit(statorLimitConfig);
      // Ramping motor output to prevent instantaneous directional changes
      // (Values need testing)
      motor.configOpenloopRamp(rampRate, timeoutMs);
    });

    // Same as set invert = false/gr
    TalonFXInvertType leftInvert = TalonFXInvertType.Clockwise;

    // Same as set invert = true
    TalonFXInvertType rightInvert = TalonFXInvertType.CounterClockwise;

    leftMaster.setInverted(leftInvert);
    rightMaster.setInverted(rightInvert);

    rightMotor2.follow(rightMaster);
    rightMotor2.setInverted(InvertType.FollowMaster);
    rightMotor3.follow(rightMaster);
    rightMotor3.setInverted(InvertType.FollowMaster);

    leftMotor2.follow(leftMaster);
    leftMotor2.setInverted(InvertType.FollowMaster);
    leftMotor3.follow(leftMaster);
    leftMotor3.setInverted(InvertType.FollowMaster);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,
      timeoutMs);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,
      timeoutMs);

    navX = new AHRS();

    drive = new DifferentialDrive(leftMaster, rightMaster);

    m_Odometry = new DifferentialDriveOdometry(navX.getRotation2d());

    shifter = new DoubleSolenoid(Ctake.MODULE_NUMBER, PneumaticsModuleType.REVPH,
      DriveConstants.SHIFTER_FORWARD_CHANNEL, DriveConstants.SHIFTER_REVERSE_CHANNEL);

  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  // *****************************************
  // ************** Driving ******************
  // *****************************************

  public void drive(double left, double right) {
    drive.tankDrive(left, right);
  }

  public void stopMotors() {
    drive.stopMotor();
  }

  public void shiftGears() {
    Value oppValue = shifter.get() == Value.kForward ? Value.kReverse : Value.kForward;
    shifter.set(oppValue);
  }

  // *****************************************
  // ************* Robot Angle ***************
  // *****************************************

  public double getAngle() {
    return navX.getAngle() % 360;
  }

  // Return robot heading in degrees, from -180 to 180

  public double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  public void zeroHeading() {
    navX.reset();
  }

  public double getTurnRate() {
    return -navX.getRate();
  }

  // ********************************************
  // ************ Odometry Functions ************
  // ********************************************

  public void updateOdometry() {
    m_Odometry.update(navX.getRotation2d(), leftMaster.getSelectedSensorPosition(),
      rightMaster.getSelectedSensorPosition());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftMaster.getSelectedSensorVelocity(),
      rightMaster.getSelectedSensorVelocity());
  }

  public void resetOdometry() {
    resetEncoders();
    m_Odometry.resetPosition(getPose(), navX.getRotation2d());
  }

  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();
  }

  // *****************************************
  // ************** Encoders *****************
  // *****************************************

  public double getAverageEncoderDistance() {
    return (leftMaster.getSelectedSensorPosition() + rightMaster
      .getSelectedSensorPosition()) / 2.0;
  }

  public double getLeftEndocderValue() {
    return leftMaster.getSelectedSensorPosition();
  }

  public double getRightEncoderValue() {
    return rightMaster.getSelectedSensorPosition();
  }

  public void resetEncoders() {
    rightMaster.setSelectedSensorPosition(0);
    leftMaster.setSelectedSensorPosition(0);
  }

  // *****************************************
  // ************** Voltage ******************
  // *****************************************

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
    drive.feed();
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("DriveTrain");
    builder.addDoubleProperty("Angle", this::getAngle, null);
    builder.addDoubleProperty("ramp rate", () -> rampRate, r -> {
      rampRate = r;
      allMotors.forEach(motor -> motor.configOpenloopRamp(rampRate, timeoutMs));
    });
  }
}
