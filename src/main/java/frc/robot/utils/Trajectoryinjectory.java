package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import java.io.IOException;
import java.util.List;

public final class Trajectoryinjectory {

  private static final TrajectoryConfig MAX_TRAJECTORY_SPEED =
      new TrajectoryConfig(
              AutoConstants.kMaxSpeedMetersPerSecond,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          .setKinematics(DriveConstants.kDriveKinematics);

  private static final Trajectory NPC_TRAJECTORY =
      TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          new Pose2d(3, 0, new Rotation2d(0)),
          MAX_TRAJECTORY_SPEED);

  public static Trajectory getTrajectory(String path) {
    try {
      return TrajectoryUtil.fromPathweaverJson(Constants.DEPLOY_DIRECTORY.resolve(path));
    } catch (IOException exception) {
      DriverStation.reportError("Error opening \"" + path + "\"", exception.getStackTrace());

      System.err.print("Error opening \"" + path + "\"");
      System.out.print(exception.getStackTrace());
      System.exit(4571);

      return null;
    }
  }

  public static Trajectory getTrajectory(String... paths) {
    Trajectory trajectory = getTrajectory(paths[0]);

    for (int i = 1; i < paths.length; ++i) {

      trajectory = trajectory.concatenate(getTrajectory(paths[i]));
    }
    return trajectory;
  }
}
