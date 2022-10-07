// Copyright (c) Darius and other Darius contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file and DARI lisence in the root directory of this
// project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.Ctake;
import frc.robot.subsystems.Intake;

public class Intakeball extends CommandBase {
  Intake intake;

  public Intakeball(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeMotor(Ctake.INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
