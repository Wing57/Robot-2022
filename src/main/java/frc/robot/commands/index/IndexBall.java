package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Constants;
import frc.robot.subsystems.Index;

public class IndexBall extends InstantCommand {

  Index index;

  public IndexBall(Index i) {

    index = i;
    addRequirements(index);
  }

  public void initialize() {

    index.setIndexMotor(Constants.INDEX_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    index.stop();
  }


}


