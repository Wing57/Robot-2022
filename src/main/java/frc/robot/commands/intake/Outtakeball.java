// Copyright (c) Darius and other Darius contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file and DARI lisence in the root directory of this
// project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.Ctake;
import frc.robot.subsystems.Intake;

public class Outtakeball extends CommandBase {
  private final Intake intake;

  public Outtakeball(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.setIntakeMotor(-Ctake.INTAKE_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
