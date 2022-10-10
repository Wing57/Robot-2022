// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTrainRamsete extends CommandBase {

  protected Boolean resetPosition;
  protected DriveTrain driveTrain;
  protected Trajectory trajectory;

  public DriveTrainRamsete(DriveTrain driveTrain, Trajectory trajectory) {
    /*  super(trajectory,
    driveTrain::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(
        DriveConstants.ksVolts,
        DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics,
    driveTrain::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    driveTrain::tankDriveVolts,
    driveTrain);

    this.resetPosition = true;
    this.trajectory = trajectory;
    this.driveTrain = driveTrain;
    */
  }

  public DriveTrainRamsete(DriveTrain driveTrain, String path) {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
