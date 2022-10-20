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
    double rawSpeed = Shooters.SHOOTER_SPEED_INTERPOLATOR.getInterpolatedValue(distance);
    double rpm = shooter.convertRawToRPM(rawSpeed);

    // align with the hub and then shoot
    addCommands(new FaceHub(drive), new SetShooterRPM(shooter, rpm));
  }
}
