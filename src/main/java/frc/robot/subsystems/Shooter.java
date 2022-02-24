// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
	WPI_TalonFX shooter1;
	WPI_TalonFX shooter2;

	public Shooter() {
		shooter1 = new WPI_TalonFX(Constants.SHOOTER1);
		shooter2 = new WPI_TalonFX(Constants.SHOOTER2);

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void shootBall(double speed) {
		shooter1.set(speed);
		shooter2.set(speed);
	}

	public void stop() {
		shooter1.set(0);
		shooter2.set(0);
	}
}
