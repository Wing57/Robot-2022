// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

  // initializing default speeds
  private double shooterPower = 0;
  private double backspinPower = 0;

  public Shooter() {

    shooterMotor = new WPI_TalonFX(Shooters.SHOOTER_MOTOR);
    backSpinMotor = new WPI_TalonFX(Shooters.BACKSPIN_MOTOR);

    bothMotors = Arrays.asList(shooterMotor, backSpinMotor);

    bothMotors.forEach(
        motor -> {
          // Factory Resets all TalonFX
          motor.configFactoryDefault();

          // Sets the motor state as either brake or coast
          motor.setNeutralMode(NeutralMode.Brake);
          // setting sensor mode
          motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        });

    backSpinInvert = TalonFXInvertType.Clockwise;
    backSpinMotor.setInverted(backSpinInvert);

    shooterInvert = TalonFXInvertType.Clockwise;
    shooterMotor.setInverted(shooterInvert);
  }

  public void setShooterVelocity(double speed) {
    SimpleMotorFeedforward feedforward = frc.robot.Constants.SFF.getShooterFF();
    double feedVoltage = feedforward.calculate(speed);
    shooterMotor.set(feedVoltage);
  }

  public void setShooterVoltage(double v) {
    shooterMotor.setVoltage(v);
  }

  public double getShooterRawVelocity() {
    return shooterMotor.getSelectedSensorVelocity();
  }

  public double getBackSpinRawVelocity() {
    return backSpinMotor.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {}

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

  public double convertRawToRPM(double ticksPer100ms) {
    return ticksPer100ms * 600.0 / 2048.0;
  }

  public double convertRPMToRaw(double rpm) {
    return rpm * 2048.0 / 600.0;
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
    builder.addDoubleProperty("Shooter RPM", () -> convertRawToRPM(getShooterRawVelocity()), null);
    builder.addDoubleProperty(
        "BackSpin RPM", () -> convertRawToRPM(getBackSpinRawVelocity()), null);
  }
}
