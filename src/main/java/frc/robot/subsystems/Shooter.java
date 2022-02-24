// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
	private final WPI_TalonFX shooterMotor, followerMotor;

	public Shooter() {
		shooterMotor = new WPI_TalonFX(Constants.SHOOTER1);
		followerMotor = new WPI_TalonFX(Constants.SHOOTER2);

		followerMotor.follow(shooterMotor);
		followerMotor.setInverted(InvertType.OpposeMaster);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void setShooterSpeed(double speed) {
		shooterMotor.set(speed);
	}

	public void stopMotors() {
		shooterMotor.set(0);
	}
}
