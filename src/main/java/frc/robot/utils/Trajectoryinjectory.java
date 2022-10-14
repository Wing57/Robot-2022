package frc.robot.utils;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import java.io.IOException;

public final class Trajectoryinjectory {

  public static Trajectory getTrajectory(String path, double maxVel, double maxAccel)
      throws IOException {
    return PathPlanner.loadPath(path, 3.048, 4.328);
  }

  public static Trajectory getTrajectory(String... paths) {
    Trajectory trajectory = getTrajectory(paths[0]);

    for (int i = 1; i < paths.length; ++i) {

      trajectory = trajectory.concatenate(getTrajectory(paths[i]));
    }
    return trajectory;
  }
}
