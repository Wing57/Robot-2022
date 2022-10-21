// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final PIDController shooterFeedback;
  private final PIDController backSpinFeedback;
  private final SimpleMotorFeedforward sff;
  private final SimpleMotorFeedforward bff;
  private final MotorController shooterController;
  private final MotorController backspinController;

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

    sff = new SimpleMotorFeedforward(Constants.SFF.Ks, Constants.SFF.Kv, Constants.SFF.Ka);
    shooterFeedback = new PIDController(ksP, ksI, ksD);
    shooterController = new MotorController(shooterMotor, sff, shooterFeedback);

    // TODO: URGENT use the ff values from sysid for backspin
    bff = new SimpleMotorFeedforward(Constants.SFF.Ks, Constants.SFF.Kv, Constants.SFF.Ka);
    backSpinFeedback = new PIDController(kbP, kbI, kbD);
    backspinController = new MotorController(backSpinMotor, bff, backSpinFeedback);
  }

  public void setShooterVoltage(double v) {
    shooterMotor.setVoltage(v);
  }

  public double getShooterRawVelocity() {
    return shooterController.getRawSpeed();
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

  @Override
  public void periodic() {
    SmartDashboard.putData("shooter/shooter PID", shooterFeedback);
    SmartDashboard.putData("shooter/backspin PID", backSpinFeedback);
    SmartDashboard.putData("shooter/shooter controller", shooterController);
    SmartDashboard.putData("shooter/backspin controller", backspinController);
  }

  /*** SHOOTER CONTROL ***/
  
  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  public void setShooterRPM(double rpm) {
    shooterController.setRPM(rpm);
  }

  public void setBackspinRPM(double rpm) {
    backspinController.setRPM(rpm);
  }

  public void setBackSpinSpeed(double speed) {
    backSpinMotor.set(speed);
  }

  public void stop() {
    shooterMotor.set(0);
    backSpinMotor.set(0);
  }

  public void runControllers() {
    shooterController.run();
    backspinController.run();
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
  }
}
