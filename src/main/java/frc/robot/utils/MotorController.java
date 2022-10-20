package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class MotorController implements Sendable {
  private final WPI_TalonFX motor;
  private final SimpleMotorFeedforward feedforward;
  private final PIDController feedback;

  private double targetRPM;

  public MotorController(
      WPI_TalonFX motor, SimpleMotorFeedforward feedforward, PIDController feedback) {
    this.motor = motor;
    this.feedforward = feedforward;
    this.feedback = feedback;
  }

  public double getRawSpeed() {
    return motor.getSelectedSensorVelocity();
  }

  public double convertRawToRPM(double ticksPer100ms) {
    return ticksPer100ms * 600.0 / 2048.0;
  }

  public double convertRPMToRaw(double rpm) {
    return rpm * 2048.0 / 600.0;
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public double getRPM() {
    return convertRawToRPM(getRawSpeed());
  }

  public void setRPM(double rpm) {
    // 5º tolerance
    feedback.setTolerance(convertRPMToRaw(rpm) * 0.05);
    targetRPM = rpm;
  }

  public void run() {
    double rawSetpoint = convertRPMToRaw(targetRPM);
    double rps = getRPM() / 60;
    double ff = feedforward.calculate(rps);
    double fb = feedback.calculate(getRawSpeed(), rawSetpoint);

    setVoltage(ff + fb);
  }

  public boolean isAtSpeed() {
    return feedback.atSetpoint();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("MotorController");
    builder.addDoubleProperty("raw speed", this::getRawSpeed, null);
    builder.addDoubleProperty("rpm", this::getRPM, null);
    builder.addDoubleProperty("target RPM", () -> this.targetRPM, null);
    builder.addBooleanProperty("is at target Speed", this::isAtSpeed, null);
  }
}
