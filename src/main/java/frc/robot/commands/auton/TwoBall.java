package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.IntakeBallForever;
import frc.robot.commands.intake.intakeExtend;
import frc.robot.commands.shooter.TarmacShot;

public class TwoBall extends SequentialCommandGroup {

  public TwoBall(RobotContainer container) {
    addCommands(
        new TarmacShot(container.shooter),
        new intakeExtend(container.intake),
        new WaitCommand(AutoConstants.INTAKE_EXTEND),
        new IntakeBallForever(container.intake),
        new WaitCommand(AutoConstants.SHOOTER_INITIALIZE));

    addCommands(new DriveTimedCommand(container.driveTrain, 1));
  }
}
