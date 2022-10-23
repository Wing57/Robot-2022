package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeBallToggle extends CommandBase {
  private final Intake intake;

  public IntakeBallToggle(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.extend();
    intake.setIntakeMotor(-Constants.Ctake.INTAKE_SPEED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.retract();
    intake.stop();
  }
}
