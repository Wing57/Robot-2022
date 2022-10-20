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
    double shooterRawSpeed = Shooters.SHOOTER_SPEED_INTERPOLATOR.getInterpolatedValue(distance);
    double backspinRawSpeed = Shooters.BACKSPIN_SPEED_INTERPOLATOR.getInterpolatedValue(distance);
    double shooterRPM = Shooter.convertRawToRPM(shooterRawSpeed);
    double backspinRPM = Shooter.convertRawToRPM(backspinRawSpeed);

    // align with the hub and then shoot
    addCommands(new FaceHub(drive), new SetShooterRPM(shooter, shooterRPM, backspinRPM));
  }
}
