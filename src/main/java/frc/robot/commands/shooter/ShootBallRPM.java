package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class ShootBallRPM extends CommandBase {
  private final Shooter shooter;
  private final Index index;

  private final Timer atSpeedTimer;
  private final double shooterRPM, backspinRPM;
  private boolean isAtSpeed;

  public ShootBallRPM(Shooter shooter, Index index, double shooterRPM, double backspinRPM) {
    this.shooter = shooter;
    this.index = index;
    this.shooterRPM = shooterRPM;
    this.backspinRPM = backspinRPM;

    addRequirements(shooter, index);

    atSpeedTimer = new Timer();
  }

  @Override
  public void initialize() {
    atSpeedTimer.reset();
    isAtSpeed = false;
  }

  @Override
  public void execute() {
    shooter.setShooterRPM(shooterRPM);
    shooter.setBackspinRPM(backspinRPM);

    boolean currentAtSpeed = shooter.isBothAtSpeed();

    if (!isAtSpeed && currentAtSpeed) {
      atSpeedTimer.start();
    } else if (isAtSpeed && !currentAtSpeed) {
      atSpeedTimer.reset();
    }

    isAtSpeed = currentAtSpeed;

    if (atSpeedTimer.get() > 2) {
      index.setIndexMotor(Constants.INDEX_SPEED);
    } else {
      index.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    index.stop();
  }
}
