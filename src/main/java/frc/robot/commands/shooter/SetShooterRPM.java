package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Vision;
import frc.robot.subsystems.Shooter;

public class SetShooterRPM extends CommandBase {
  private final Shooter shooter;

  private final double shooterRPM;
  private final double backspinRPM;

  public SetShooterRPM(Shooter shooter, double shooterRPM, double backspinRPM) {
    this.shooter = shooter;
    this.shooterRPM = shooterRPM;
    this.backspinRPM = backspinRPM;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    this.shooter.setShooterRPM(shooterRPM);
    this.shooter.setBackspinRPM(backspinRPM);
  }

  @Override
  public void execute() {
    shooter.runControllers();
  }

  @Override
  public boolean isFinished() {
    return !Vision.getInstance().isFacingHub();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    shooter.stopBackSpinMotor();
  }
}
