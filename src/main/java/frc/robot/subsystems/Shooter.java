// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Shooters;
import java.util.Arrays;
import java.util.List;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX shooterMotor, backSpinMotor;
  private final TalonFXInvertType backSpinInvert;
  private final TalonFXInvertType shooterInvert;

  private final List<WPI_TalonFX> bothMotors;

  private final PIDFlywheel shooter;

  private final SmartNumber m_targetVelocity;
  private final IFilter targetFilter;

  // initializing default speeds
  private double shooterPower = 0;
  private double backspinPower = 0;

  public Shooter() {
    m_targetVelocity = new SmartNumber("Target Velocity", 0.0);

    // Basically a better ramprate; reduces encoder noise/motor jerk without introducing much delay
    targetFilter = new LowPassFilter(Shooters.RC);

    shooterMotor = new WPI_TalonFX(Shooters.SHOOTER_MOTOR);
    backSpinMotor = new WPI_TalonFX(Shooters.BACKSPIN_MOTOR);

    bothMotors = Arrays.asList(shooterMotor, backSpinMotor);

    bothMotors.forEach(
        motor -> {
          // Factory Resets all TalonFX
          motor.configFactoryDefault();

          // Sets the motor state as either brake or coast
          motor.setNeutralMode(NeutralMode.Coast);

          // setting sensor mode
          motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        });

    backSpinInvert = TalonFXInvertType.Clockwise;
    backSpinMotor.setInverted(backSpinInvert);

    shooterInvert = TalonFXInvertType.Clockwise;
    shooterMotor.setInverted(shooterInvert);

    shooter =
        new PIDFlywheel(
            shooterMotor, Shooters.ShooterFF.getController(), Shooters.ShooterPID.getController());
  }

  /*** Shooter Control ***/

  public void setShooterVelocity(double speed) {
    m_targetVelocity.set(speed);
  }

  /*** Encoder Readings ***/

  public double getShooterRawVelocity() {
    return shooterMotor.getSelectedSensorVelocity();
  }

  public double getBackSpinRawVelocity() {
    return backSpinMotor.getSelectedSensorVelocity();
  }

  // This automatically converts the Talon FX raw 100 ticks per ms into RPM
  public double getShooterRPM() {
    return shooter.getVelocity();
  }

  /*** Shooter Readings ***/

  public double getRawTargetVelocity() {
    return m_targetVelocity.get();
  }

  public double getTargetVelocity() {
    return targetFilter.get(getRawTargetVelocity());
  }

  public boolean isReady() {
    return Math.abs(getShooterRPM() - getTargetVelocity()) < Shooters.MAX_SPEED_ERROR;
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  public void setBackSpinSpeed(double speed) {
    backSpinMotor.set(speed);
  }

  public void stopShooter() {
    shooterMotor.set(0);
  }

  public void stopBackSpinMotor() {
    backSpinMotor.set(0);
  }

  @Override
  public void periodic() {
    double setpoint = getTargetVelocity();

    if (setpoint < Shooters.MIN_RPM) {
      shooter.stop();
    } else {
      shooter.setVelocity(setpoint);
    }
  }

  // spotless:off
  /**
   * ticks     10 100ms     60s        1 rev
   * ------ x --------- x ------- x ------------
   * 100ms        1s        1min     2048 ticks
   *
   * @param ticksPer100ms
   * @return vel in RPM
   */
  // spotless:on
  public double convertRawToRPM(double ticksPer100ms) {
    return ticksPer100ms * 600.0 / 2048.0;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Shooter Power",
        () -> shooterPower,
        p -> {
          shooterPower = p;
          setShooterSpeed(shooterPower);
        });
    builder.addDoubleProperty(
        "Backspin Power",
        () -> backspinPower,
        p -> {
          backspinPower = p;
          setBackSpinSpeed(backspinPower);
        });
    builder.addDoubleProperty("Raw Shooter Vel", this::getShooterRawVelocity, null);
    builder.addDoubleProperty("Raw BackSpinSpeed", this::getBackSpinRawVelocity, null);
    builder.addDoubleProperty("Shooter RPM", this::getShooterRPM, null);
  }
}
