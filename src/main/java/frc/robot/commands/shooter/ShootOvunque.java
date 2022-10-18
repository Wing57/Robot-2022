// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import com.stuypulse.stuylib.streams.booleans.BStream;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision;
import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Shooter;

public class ShootOvunque extends CommandBase {

  Shooter m_shooter;
  Vision m_vision;

  BStream shooterReady;

  public ShootOvunque(Shooter shooter, Vision vision) {
    m_shooter = shooter;
    m_vision = vision;

    shooterReady = BStream.create(() -> shooter.isReady())
    .and(() -> vision.hasValidTarget());

    addRequirements(shooter);
  }

  @Override
  public void execute() {
    if (m_vision.hasValidTarget()) {
      
      double distance = m_vision.getEstimatedDistance();
      m_shooter.setShooterVelocity(Shooters.SHOOTER_SPEED_INTERPOLATOR.getInterpolatedValue(distance));
      m_shooter.setBackSpinVelocity(Shooters.BACKSPIN_SPEED_INTERPOLATOR.getInterpolatedValue(distance));
    }
      else {
        System.out.println("No Target Innit");
    }
  }
}