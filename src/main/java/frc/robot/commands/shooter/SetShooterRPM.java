package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class SetShooterRPM extends CommandBase {
  private final Shooter shooter;
  // TODO: figure out these values
  private final double kP = 0.0, kI = 0.0, kD = 0.0;
  private final PIDController controller;
  private final SimpleMotorFeedforward ff;
  private final double setpointRPM;

  private double currentRawSpeed, rpm, rps, rawSetpoint;

  public SetShooterRPM(Shooter shooter, double setpointRPM) {
    this.shooter = shooter;
    this.setpointRPM = setpointRPM;
    addRequirements(this.shooter);
    controller = new PIDController(kP, kI, kD);
    // 5% tolerance
    rawSetpoint = shooter.convertRPMToRaw(setpointRPM);
    controller.setTolerance(rawSetpoint * 0.05);
    ff = Constants.SFF.getShooterFF();
  }

  @Override
  public void initialize() {
    controller.setSetpoint(rawSetpoint);
  }

  private void log() {
    SmartDashboard.putNumber("shootRPM/setpoint (raw)", rawSetpoint);
    SmartDashboard.putNumber("shootRPM/raw speed", currentRawSpeed);
    SmartDashboard.putNumber("shootRPM/setpoint (rpm)", setpointRPM);
    SmartDashboard.putNumber("shootRPM/RPM", rpm);
    SmartDashboard.putData("shootRPM/pidController", controller);
  }

  @Override
  public void execute() {
    currentRawSpeed = shooter.getShooterRawVelocity();
    rpm = shooter.convertRawToRPM(currentRawSpeed);
    rps = rpm / 60;
    shooter.setShooterVoltage(controller.calculate(currentRawSpeed) + ff.calculate(rps));
    log();
  }

  @Override
  public boolean isFinished() {
    // command wont work if it isnt facing the hub
    return !shooter.isFacingHub();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }
}
