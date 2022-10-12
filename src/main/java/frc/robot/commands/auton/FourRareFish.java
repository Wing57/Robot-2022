// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveTrainRamsete;
import frc.robot.commands.intake.IntakeBallForever;
import frc.robot.commands.intake.intakeExtend;
import frc.robot.commands.shooter.TarmacShot;

public class FourRareFish extends SequentialCommandGroup {

  private static final String FOUR_FISH_SWIM = "deploy/paths/2fishcuh.wpilib.json";
  private static final String FOUR_FISH_GLUP = "deploy/paths/2fishglup.wpilib.json";
  private static final String FOUR_FISH_DROWN = "deploy/paths/4fishcuh.wpilib.json";

  public FourRareFish(RobotContainer container) {

    // Initialize Subsytems
    addCommands(
        new TarmacShot(container.shooter),
        new intakeExtend(container.intake),
        new WaitCommand(AutoConstants.INTAKE_EXTEND),
        new IntakeBallForever(container.intake),
        new WaitCommand(AutoConstants.SHOOTER_INITIALIZE));

    // Get first ball the line up to shoot
    addCommands(new DriveTrainRamsete(container.driveTrain, FOUR_FISH_SWIM).robotRelative());

    addCommands();
  }
}
