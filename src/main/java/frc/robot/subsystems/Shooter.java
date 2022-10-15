// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

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
        });

    backSpinInvert = TalonFXInvertType.Clockwise;
    backSpinMotor.setInverted(backSpinInvert);

    shooterInvert = TalonFXInvertType.Clockwise;
    shooterMotor.setInverted(shooterInvert);
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
        () -> Shooters.SHOOT_SPEED,
        ss -> {
          ss = Shooters.SHOOT_SPEED;
          setShooterSpeed(ss);
        });
    builder.addDoubleProperty(
        "Backspin Speed",
        () -> Shooters.BACKSPIN_SPEED,
        bs -> {
          bs = Shooters.BACKSPIN_SPEED;
          setBackSpinSpeed(bs);
        });
  }
}
