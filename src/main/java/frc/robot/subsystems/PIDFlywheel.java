// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.stuypulse.stuylib.control.Controller;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDFlywheel extends SubsystemBase {

  private final double m_targetVelocity;

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

  @Override
  public void periodic() {}
}
