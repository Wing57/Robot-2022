// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Shooters;
import java.util.Arrays;
import java.util.List;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX shooterMotor, backSpinMotor;
  private final TalonFXInvertType backSpinInvert;
  private final TalonFXInvertType shooterInvert;

  private final List<WPI_TalonFX> bothMotors;

  // initializing default speeds
  private double shooterSpeed = 0;
  private double backspinSpeed = 0;
  private final SmartNumber m_targetVelocity;

  public Shooter() {
    m_targetVelocity = new SmartNumber("Target Velocity", 0.0);

    shooterMotor = new WPI_TalonFX(Shooters.SHOOTER_MOTOR);
    backSpinMotor = new WPI_TalonFX(Shooters.BACKSPIN_MOTOR);

    bothMotors = Arrays.asList(shooterMotor, backSpinMotor);

    bothMotors.forEach(
        motor -> {
          // Factory Resets all TalonFX
          motor.configFactoryDefault();

          // Sets the motor state as either brake or coast
          motor.setNeutralMode(NeutralMode.Brake);
        });

    backSpinInvert = TalonFXInvertType.Clockwise;
    backSpinMotor.setInverted(backSpinInvert);

    shooterInvert = TalonFXInvertType.Clockwise;
    shooterMotor.setInverted(shooterInvert);
  }

  public void setShooterVelocity(double speed) {
    SmartDashboard.putNumber("Desired Speed", speed);

    m_targetVelocity.set(speed);

    // TODO: Find actual ff gains
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 0.2, 0.3);
    double feedVoltage = feedforward.calculate(speed);

    shooterMotor.setVoltage(feedVoltage);
  }

  public double getShooterVelocity() {
    return shooterMotor.getSelectedSensorVelocity();
  }

  public double getBackSpinVelocity() {
    return backSpinMotor.getSelectedSensorVelocity();
  }

  public double getTargetVelocity() {
    return m_targetVelocity.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  public void setBackSpinSpeed(double speed) {
    backSpinMotor.set(speed);
  }

  // public void setTurretSpeed(double speed) {
  // turretMotor.set(speed);
  // }

  public void stopShooter() {
    shooterMotor.set(0);
  }

  public void stopBackSpinMotor() {
    backSpinMotor.set(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Shooter Speed",
        () -> shooterSpeed,
        speed -> {
          shooterSpeed = speed;
          setShooterSpeed(speed);
        });
    builder.addDoubleProperty(
        "Backspin Speed",
        () -> backspinSpeed,
        speed -> {
          backspinSpeed = speed;
          setBackSpinSpeed(backspinSpeed);
        });
    builder.addDoubleProperty("Actual ShooterSpeed", this::getShooterVelocity, null);
    builder.addDoubleProperty("Actual BackSpinSpeed", this::getBackSpinVelocity, null);
  }
}
