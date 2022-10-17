// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDFlywheel extends SubsystemBase {

  private double m_targetVelocity;

  private final WPI_TalonFX shooterMotor;

  private final SimpleMotorFeedforward feedforward;
  private final Controller feedback;

  public PIDFlywheel(
      WPI_TalonFX shooterMotor, SimpleMotorFeedforward feedforward, Controller feedback) {
    this.feedforward = feedforward;
    this.feedback = feedback;

    this.shooterMotor = shooterMotor;

    this.m_targetVelocity = 0.0;
  }

  public void setVelocity(double m_targetVelocity) {
    this.m_targetVelocity = m_targetVelocity;
  }

  public void stop() {
    setVelocity(0);
  }

  public double getVelocity() {
    return shooterMotor.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
    if (this.m_targetVelocity < 200) {

      shooterMotor.stopMotor();

    } else {
      double ff = feedforward.calculate(this.m_targetVelocity);
      double fb = feedback.update(this.m_targetVelocity, getVelocity());

      shooterMotor.setVoltage(SLMath.clamp(ff + fb, 0, 16));
    }
  }
}
