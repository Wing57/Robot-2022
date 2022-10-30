// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveTrainRamsete;
import frc.robot.commands.index.IndexBall;
import frc.robot.commands.intake.IntakeBallForever;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.shooter.TarmacShot;

public class FourRareFish extends SequentialCommandGroup {

  private static final String FOUR_FISH_SWIM = "paths/2fishcuh.wpilib.json";
  private static final String FOUR_FISH_GLUP = "paths/2fishglup.wpilib.json";
  private static final String FOUR_FISH_DROWN = "paths/4fishcuh.wpilib.json";

  private static final PathPlannerTrajectory FIRST_FISH_CUH =
      PathPlanner.loadPath("FIRST_FISH_CUH", 0, 0);

  public FourRareFish(RobotContainer container) {

    // Initialize Subsytems
    addCommands(
        new TarmacShot(container.shooter),
        new IntakeExtend(container.intake),
        new WaitCommand(AutoConstants.INTAKE_EXTEND),
        new IntakeBallForever(container.intake),
        new WaitCommand(AutoConstants.SHOOTER_INITIALIZE));

    // Get first ball the line up to shoot
    addCommands(new DriveTrainRamsete(container.driveTrain, FOUR_FISH_SWIM).robotRelative());

    // This should shoot bofa balls out bc shooter is already running
    addCommands(new IndexBall(container.index).withTimeout(AutoConstants.INDEX_BALL));

    addCommands(
        new DriveTrainRamsete(container.driveTrain, FOUR_FISH_GLUP).fieldRelative(),
        new IndexBall(container.index)
            .withTimeout(AutoConstants.HUMAN_WAIT + AutoConstants.INDEX_BALL));

    // Return to tarmac and shoot remaining balls
    addCommands(
        new DriveTrainRamsete(container.driveTrain, FOUR_FISH_DROWN).fieldRelative(),
        new IndexBall(container.index).withTimeout(AutoConstants.INDEX_BALL));
  }
}
