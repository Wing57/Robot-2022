// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import com.stuypulse.stuylib.streams.booleans.BStream;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.Shooters;
import frc.robot.Vision;
import frc.robot.subsystems.Shooter;

public class ShootOvunque extends CommandBase {

  private final Shooter shooter;
  private final Vision vision = Vision.getInstance();

  BStream shooterReady;

  public ShootOvunque(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void execute() {
    if (vision.hasValidTarget()) {
      double distance = vision.getEstimatedDistance();
      shooter.setShooterVelocity(
          Shooters.SHOOTER_SPEED_INTERPOLATOR.getInterpolatedValue(distance));
    } else {
      System.out.println("No Target Innit");
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    shooter.stopBackSpinMotor();
  }
}
