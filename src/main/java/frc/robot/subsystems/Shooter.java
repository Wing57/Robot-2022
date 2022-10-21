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

import frc.robot.Constants;
import frc.robot.Constants.Shooters;
import frc.robot.utils.MotorController;
import java.util.Arrays;
import java.util.List;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX shooterMotor, backSpinMotor;
  private final TalonFXInvertType backSpinInvert;
  private final TalonFXInvertType shooterInvert;

  private final List<WPI_TalonFX> bothMotors;

  // TODO: tune these values
  // shooter pid
  private final double ksP = 0.144, ksI = 0.0, ksD = 0.0;
  // backspin pid
  private final double kbP = 0.04993, kbI = 0.0, kbD = 0.0;

  private final SimpleMotorFeedforward sff;
  private final SimpleMotorFeedforward bff;
  private final MotorController shooterController;
  private final MotorController backspinController;

  private final int timeoutMs = Constants.timeoutMs;

  public Shooter() {
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
          motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, timeoutMs);

          motor.configNominalOutputForward(0, timeoutMs);
          motor.configNominalOutputReverse(0, timeoutMs);
          motor.configPeakOutputForward(1, timeoutMs);
          motor.configPeakOutputReverse(-1, timeoutMs);
          motor.configClosedloopRamp(0.15, timeoutMs);
        });

    backSpinInvert = TalonFXInvertType.Clockwise;
    // TODO: make sure sensor gives positve value when motor is flashing green
    // backSpinMotor.setSensorPhase(true);
    backSpinMotor.setInverted(backSpinInvert);

    shooterInvert = TalonFXInvertType.Clockwise;
    // TODO: same thing here
    // shooterMotor.setSensorPhase(true);
    shooterMotor.setInverted(shooterInvert);

    sff = new SimpleMotorFeedforward(Constants.SFF.Ks, Constants.SFF.Kv, Constants.SFF.Ka);
    shooterController = new MotorController(shooterMotor, sff);
    shooterController.setPID(ksP, ksI, ksD);

    bff = new SimpleMotorFeedforward(Constants.BFF.Ks, Constants.BFF.Kv, Constants.BFF.Ka);
    backspinController = new MotorController(backSpinMotor, bff);
    backspinController.setPID(kbP, kbI, kbD);
  }

  public double getShooterRawVelocity() {
    return shooterMotor.getSelectedSensorVelocity();
  }

  public double getBackSpinRawVelocity() {
    return backSpinMotor.getSelectedSensorVelocity();
  }

  public static double convertRawToRPM(double ticksPer100ms) {
    return ticksPer100ms * 600.0 / 2048.0;
  }

  public static double convertRPMToRaw(double rpm) {
    return rpm * 2048.0 / 600.0;
  }

  public void resetSensors() {
    shooterMotor.setSelectedSensorPosition(0, 0, timeoutMs);
    backSpinMotor.setSelectedSensorPosition(0, 0, timeoutMs);
  }

  @Override
  public void periodic() {}

  /*** SHOOTER CONTROL ***/

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  public void setBackSpinSpeed(double speed) {
    backSpinMotor.set(speed);
  }

  public void setShooterRPM(double shooterRPM) {
    shooterController.setRPM(shooterRPM);
  }

  public void setBackspinRPM(double backspinRPM) {
    backspinController.setRPM(backspinRPM);
  }

  public void stop() {
    shooterMotor.set(0);
    backSpinMotor.set(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    shooterController.initSendable(builder);
    backspinController.initSendable(builder);
  }
}
