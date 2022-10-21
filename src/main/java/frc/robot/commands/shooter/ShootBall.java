// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Shooter;

public class ShootBall extends CommandBase {
  // Shooter declaration
  Shooter shooter;

  public ShootBall(Shooter s) {
    shooter = s;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.setShooterSpeed(Shooters.SHOOT_SPEED);
    shooter.setBackSpinSpeed(Shooters.BACKSPIN_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
