// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

public class PIDFlywheel extends SubsystemBase {

  private double m_targetVelocity;
  private double m_targetBSVelocity;

  private final List<WPI_TalonFX> motors;

  private final SimpleMotorFeedforward feedforward;
  private final Controller feedback;

  public PIDFlywheel(WPI_TalonFX motor, SimpleMotorFeedforward feedforward, Controller feedback) {
    this.feedforward = feedforward;
    this.feedback = feedback;

    this.motors = new ArrayList<>();

    this.m_targetVelocity = 0.0;
    this.m_targetBSVelocity = 0.0;
  }

  public void setVelocity(double m_targetVelocity) {
    this.m_targetVelocity = m_targetVelocity;
  }

  public void setBSVelocity(double m_targetBSVelocity) {
    this.m_targetBSVelocity = m_targetBSVelocity;
  }

  public void stop() {
    setVelocity(0);
    setBSVelocity(0);
  }

  // TODO: Could be problematic, must be tested
  public double getVelocity() {
    double velocity = 0.0;

    for (WPI_TalonFX motor : motors) {
      velocity += motor.getSelectedSensorVelocity();
    }
    return velocity * 600.0 / 2048.0;
  }

  @Override
  public void periodic() {
    if (this.m_targetVelocity < 200) {

      for (WPI_TalonFX motor : motors) {
        motor.stopMotor();
      }
    } else {
      double ff = feedforward.calculate(this.m_targetVelocity);
      double fb = feedback.update(this.m_targetVelocity, getVelocity());

      for (WPI_TalonFX motor : motors) {
        motor.setVoltage(SLMath.clamp(ff + fb, 0, 16));
      }
    }
  }
}
