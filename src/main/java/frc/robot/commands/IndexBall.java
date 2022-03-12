// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Index;

public class IndexBall extends CommandBase {
  Index index;

  public IndexBall(Index index) {
    this.index = index;
    addRequirements(index);
  }


  @Override
  public void execute() {
    index.setIndexMotor(Constants.INDEX_SPEED);
  }

 
  @Override
  public void end(boolean interrupted) {
    index.stop();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
