package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.Index;

public class IndexBall extends CommandBase {

  Index index;

  public IndexBall(Index i) {

    index = i;
    addRequirements(index);
  }

  public void execute() {

    index.setIndexMotor(Constants.INDEX_SPEED);
  }
}
