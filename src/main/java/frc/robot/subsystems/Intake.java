// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final WPI_TalonFX intakeMotor;

  public Intake() {
    intakeMotor = new WPI_TalonFX(Constants.INTAKE1);
  }

  @Override
  public void periodic() {
  }

  public void intakeBall(double speed) {
    intakeMotor.set(speed);
  }

  public void stop() {

    intakeMotor.set(0);
  }
}
