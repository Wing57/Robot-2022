package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Constants.Ctake;
import frc.robot.subsystems.Intake;

public class IntakeBallForever extends InstantCommand {

  private final Intake intake;

  public IntakeBallForever(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setIntakeMotor(-Ctake.INTAKE_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
