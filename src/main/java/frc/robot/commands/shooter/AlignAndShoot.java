package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.Shooters;
import frc.robot.Vision;
import frc.robot.commands.drive.FaceHub;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class AlignAndShoot extends SequentialCommandGroup {

  public AlignAndShoot(DriveTrain drive, Shooter shooter) {
    double distance = Vision.getInstance().getEstimatedDistance();
    double shooterRPM = Shooters.SHOOTER_SPEED_INTERPOLATOR.getInterpolatedValue(distance);
    double backspinRPM = Shooters.BACKSPIN_SPEED_INTERPOLATOR.getInterpolatedValue(distance);

    // align with the hub and then shoot
    addCommands(new FaceHub(drive), new SetShooterRPM(shooter, shooterRPM, backspinRPM));
  }
}
