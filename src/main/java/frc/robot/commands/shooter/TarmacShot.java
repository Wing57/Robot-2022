// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.Constants.Shooters;
import frc.robot.subsystems.Shooter;

public class TarmacShot extends SetShooterSpeed {
  /** Creates a new TarmacShot. */
  public TarmacShot(Shooter shooter) {
    super(shooter, Shooters.TARMAC_SPEED, Shooters.TARMAC_BSPEED);
  }
}
