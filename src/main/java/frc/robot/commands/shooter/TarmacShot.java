// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Shooter;

public class TarmacShot extends CommandBase {
  /** Creates a new TarmacShot. */
  public TarmacShot(Shooter shooter) {

    new SetShooterSpeed(shooter, Shooters.SHOOT_SPEED, Shooters.BACKSPIN_SPEED);
  }
}
