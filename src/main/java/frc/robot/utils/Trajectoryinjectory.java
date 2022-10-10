/* package frc.robot.utils;


public final class Trajectoryinjectory {

  private static final TrajectoryConfig MAX_TRAJECTORY_SPEED =
      new TrajectoryConfig(
              AutoConstants.kMaxSpeedMetersPerSecond,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          .setKinematics(DriveConstants.kDriveKinematics);

  private static final Trajectory NPC_TRAJECTORY = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    new Pose2d(3, 0, new Rotation2d(0)),
    MAX_TRAJECTORY_SPEED);
}

public static Trajectory getTrajectory(String path) {
}
*/
