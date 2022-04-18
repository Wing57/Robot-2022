// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX shooterMotor, backSpinMotor;
  // private final WPI_TalonFX turretMotor;
  private final TalonFXInvertType backSpinInvert;

  public Shooter() {
    shooterMotor = new WPI_TalonFX(Constants.SHOOTER_MOTOR_1);
    // turretMotor = new WPI_TalonFX(Constants.TURRET_MOTOR);
    backSpinMotor = new WPI_TalonFX(Constants.SHOOTER_MOTOR_2);

    shooterMotor.configFactoryDefault();
    backSpinMotor.configFactoryDefault();

    shooterMotor.setNeutralMode(NeutralMode.Coast);
    backSpinMotor.setNeutralMode(NeutralMode.Coast);

    backSpinInvert = TalonFXInvertType.Clockwise;

    backSpinMotor.setInverted(backSpinInvert);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);

  }

  public void setReverseShooterSpeed(double speed) {
    shooterMotor.set(-speed);
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

  public void stopReverseShooter() {
    shooterMotor.set(0);
  }

  public void stopBackSpinMotor() {
    backSpinMotor.set(0);
  }
}
