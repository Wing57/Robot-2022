// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
	private final WPI_TalonSRX turretMotor;

	public Turret() {
		turretMotor = new WPI_TalonSRX(0);
	}

	@Override
	public void periodic() {
	}

	public void setTurretMotor(double speed) {
		turretMotor.set(speed);
	}
}
