// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
	private final WPI_TalonFX shooterMotor, followerMotor;
	private final WPI_TalonSRX turretMotor;

	public Shooter() {
		shooterMotor = new WPI_TalonFX(Constants.SHOOTER_MOTOR_1);
		turretMotor = new WPI_TalonSRX(Constants.TURRET_MOTOR);
		followerMotor = new WPI_TalonFX(Constants.SHOOTER_MOTOR_2);

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

	public void setTurretSpeed(double speed) {
		turretMotor.set(speed);
	}

	public void stopShooter() {
		shooterMotor.set(0);
	}
}
