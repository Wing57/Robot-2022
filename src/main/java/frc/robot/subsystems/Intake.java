// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ctake;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX intakeMotor;

  private final DoubleSolenoid piston;
  private final Compressor comp;

  private final double rampRate;
  private final int timeoutMs;

  public Intake() {
    intakeMotor = new WPI_TalonSRX(Ctake.INTAKE_MOTOR);

    rampRate = 0.20;
    timeoutMs = 15;

    intakeMotor.configOpenloopRamp(rampRate, timeoutMs);

    comp = new Compressor(Ctake.MODULE_NUMBER, PneumaticsModuleType.REVPH);
    piston =
        new DoubleSolenoid(
            Ctake.MODULE_NUMBER,
            PneumaticsModuleType.REVPH,
            Ctake.INTAKE_PISTON_FORWARD_CHANNEL,
            Ctake.INTAKE_PISTON_REVERSE_CHANNEL);

    comp.enableDigital();
  }

  public double getIntakeMotor() {
    return intakeMotor.get();
  }

  public DoubleSolenoid getPiston() {
    return this.piston;
  }

  public void togglePiston() {
    Value oppositeValue = piston.get() == Value.kForward ? Value.kReverse : Value.kForward;
    piston.set(oppositeValue);
  }

  public void extend() {
    piston.set(Value.kForward);
  }

  public void retract() {
    piston.set(Value.kReverse);
  }

  public void setIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  public void close() throws Exception {
    piston.close();
    intakeMotor.close();
  }
}
