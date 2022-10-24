// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class SetShooterSpeed extends InstantCommand {

  private final Shooter shooter;
  private final double targetSpeed;
  private final double targetBackSpeed;

  public SetShooterSpeed(Shooter s, double targetSpeed, double targetBackSpeed) {
    shooter = s;
    this.targetSpeed = targetSpeed;
    this.targetBackSpeed = targetBackSpeed;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setShooterSpeed(targetSpeed);
    shooter.setBackSpinSpeed(targetBackSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
}
