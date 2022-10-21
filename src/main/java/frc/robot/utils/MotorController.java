package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;

public class MotorController implements Sendable {
  private final WPI_TalonFX motor;
  private final PIDTuner tuner;
  private final SimpleMotorFeedforward ff;

  private int kPIDLoopIdx;
  private final int timeoutMs = Constants.timeoutMs;

  private double targetRPM, toleranceRPM;

  public MotorController(WPI_TalonFX motor, SimpleMotorFeedforward ff) {
    this.motor = motor;
    this.ff = ff;
    this.tuner = new PIDTuner();
    this.tuner.setUpdater(this::updatePID);
    this.kPIDLoopIdx = 0;
  }

  public void setkPIDLoopIdx(int kPIDLoopIdx) {
    this.kPIDLoopIdx = kPIDLoopIdx;
  }

  public void setPID(double kP, double kI, double kD) {
    this.tuner.setPID(kP, kI, kD);
    setMotorPID(kP, kI, kD);
  }

  private void setMotorPID(double kP, double kI, double kD) {
    motor.config_kP(kPIDLoopIdx, kP, timeoutMs);
    motor.config_kI(kPIDLoopIdx, kI, timeoutMs);
    motor.config_kD(kPIDLoopIdx, kD, timeoutMs);
  }

  /** This syncs up the tuner PIDF values with the motor. This should be run frequently */
  public void updatePID(PIDTuner tuner) {
    setMotorPID(tuner.getkP(), tuner.getkI(), tuner.getkD());
  }

  /**
   * How much RPM can it be off by
   *
   * @param rpm
   */
  public void setTolerance(double rpm) {
    toleranceRPM = rpm;
    motor.configAllowableClosedloopError(kPIDLoopIdx, convertRPMToRaw(rpm));
  }

  public double getRawSpeed() {
    return motor.getSelectedSensorVelocity(kPIDLoopIdx);
  }

  public double convertRawToRPM(double ticksPer100ms) {
    return ticksPer100ms * 600.0 / 2048.0;
  }

  public double convertRPMToRaw(double rpm) {
    return rpm * 2048.0 / 600.0;
  }

  public double getRPM() {
    return convertRawToRPM(getRawSpeed());
  }

  public void setRPM(double rpm) {
    targetRPM = rpm;
    double rawSpeed = convertRPMToRaw(rpm);
    double rps = rpm / 60;
    motor.set(ControlMode.Velocity, rawSpeed, DemandType.ArbitraryFeedForward, ff.calculate(rps));
  }

  public boolean isAtSpeed() {
    return Math.abs(targetRPM - getRPM()) < toleranceRPM;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("MotorController");
    builder.addDoubleProperty("raw speed", this::getRawSpeed, null);
    builder.addDoubleProperty("rpm", this::getRPM, null);
    builder.addDoubleProperty("target RPM", () -> this.targetRPM, null);
    builder.addBooleanProperty("is at speed", this::isAtSpeed, null);
    tuner.initSendable(builder);
  }
}
